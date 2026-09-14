# Upstream asks for keelson

Paste-ready material for one keelson PR against `dev`. Nothing here is filed yet.

Until it lands, `keelson-processor-navigation` publishes each of these as a `raw_json` rider with
the same field set (see `src/nav/contracts/raw_json_riders.yaml`). A test fails when the installed
keelson SDK registers any of them, which is the signal to switch `codec: proto` and retire the rider.

## Why these exist

- **`navigation_state`**: nothing states what a vessel's navigation function is doing. `vehicle_mode`
  is an autopilot mode string, `operational_authority` is a ceiling, `command_authority` is a lease.
- **`encounter`, `navigation_advice`, `advice_disposition`**: crowsnest-dev
  `docs/NAVIGATION-DECISION-SERVICES.md` §6.1 asks for exactly this family. Nothing on the bus can
  say "encounter with target X, crossing from starboard, we are give-way, and the advised action is
  40° to starboard". Advice is a record, never a command, so the advisor cannot hold the conn by
  construction.

## `messages/subjects.yaml`

```yaml
# Navigation decisions (keelson-processor-navigation). Provisional.
navigation_state:                       keelson.NavigationState    # What the navigation function is doing now, 1 Hz
encounter:                              keelson.Encounter          # One vessel-to-vessel encounter, restated until it closes
navigation_advice:                      keelson.NavigationAdvice   # A record of advised action, never a command
advice_disposition:                     keelson.AdviceDisposition  # Accepted / rejected / superseded / expired, and by whom
```

## `messages/qos.yaml`

```yaml
  navigation_state: elevated
  encounter: elevated
  navigation_advice: elevated
  advice_disposition: elevated
```

## Keys and persistence

| Subject | Key after `pubsub/` | Router storage |
|---|---|---|
| `navigation_state` | `navigation_state/{source_id}` | no, latest wins |
| `encounter` | `encounter/{voyage_id or novoyage}/{encounter_id}` | yes, restated in full until closed |
| `navigation_advice` | `navigation_advice/{voyage_id or novoyage}/{advice_id}` | yes |
| `advice_disposition` | `advice_disposition/{voyage_id or novoyage}/{advice_id}` | yes, last disposition per advice |

Every id is a single key token, so `.../encounter/*/*` storage covers the tree.

## Open points to settle in review

- `Encounter.Situation` repeats `ColregSituation` from `Route.proto` and adds `OVERTAKEN` and `NONE`.
  The alternative is to add those two to `ColregSituation`, which a route plan would then carry
  without meaning.
- `NavigationAdvice` redeclares steering and propulsion proposals because a payload cannot import
  `SteeringOrder` / `PropulsionOrder` from the unmerged `feature/standing-order-verb` interface. If
  those move to the payload pool, use them.
- `RouteExecution.speed_over_ground_knots` and `course_over_ground_deg` are not `optional`, so a
  producer with no SOG publishes 0 kn. Presence is wanted.
- `route_execution` for a vessel's declared route (no voyage) is keyed `declared-{source}` today.

## `messages/payloads/NavigationState.proto`

```proto
// DRAFT for keelson/messages/payloads. Not yet proposed upstream.
//
// Subject:  navigation_state: keelson.NavigationState
// Key:      {base_path}/@v0/{entity_id}/pubsub/navigation_state/{source_id}
// QoS:      elevated
// Persist:  no (latest wins, 1 Hz)
//
// Until keelson registers it, keelson-processor-navigation publishes the same field set as a
// raw_json rider (record "navigation_state"). src/nav/conduct/state.py mirrors this file and
// tests/test_state.py fails on drift.

syntax = "proto3";

import "google/protobuf/timestamp.proto";
import "Route.proto";                 // RouteRef
import "OperationalAuthority.proto";  // AuthorityLevel

package keelson;

// What the vessel's navigation function is doing right now, stated by the process doing it.
//
// Not a lock and not a grant. command_authority says who holds the conn, operational_authority
// says the most the vessel may claim, and this says what navigation made of both. A consumer
// deciding whether to act on the vessel reads those, never this.
message NavigationState {

  enum Mode {
    MODE_UNSPECIFIED = 0;
    // No active voyage, or none this process can resolve.
    MODE_IDLE = 1;
    // A voyage is active and monitored. Not holding the conn, commanding nothing.
    MODE_READY = 2;
    // Holding the conn and commanding the vessel along the voyage.
    MODE_UNDERWAY = 3;
    // Lost something it needs to navigate (conn, authority, fresh ownship state) and has
    // commanded its declared safe state. `reason` and `degraded_inputs` say what.
    MODE_FALLBACK = 4;
    // Reached the final waypoint of the active voyage.
    MODE_ARRIVED = 5;
    // The voyage was completed or aborted; conn released.
    MODE_ENDED = 6;
  }

  // Refines MODE_UNDERWAY; UNSPECIFIED in every other mode.
  enum SubMode {
    SUB_MODE_UNSPECIFIED = 0;
    SUB_MODE_TRACK = 1;      // following the planned track
    SUB_MODE_AVOIDING = 2;   // executing accepted collision-avoidance advice
    SUB_MODE_HOLDING = 3;    // speed zero or loitering, waiting on a condition
    SUB_MODE_REROUTING = 4;  // awaiting a new route edition
  }

  google.protobuf.Timestamp timestamp = 1;
  Mode mode = 2;
  SubMode sub_mode = 3;

  // The voyage being monitored or sailed. Absent in MODE_IDLE.
  optional string voyage_id = 4;

  // The route edition in force. Absent when no edition is resolved; never `latest`.
  optional RouteRef route_ref = 5;

  // The leg being sailed, named by the waypoint it leaves (RouteExecution convention).
  optional string current_leg_from_waypoint_id = 6;

  // The operational_authority level this process is acting under. UNKNOWN when it has heard
  // none, which is non-authorising exactly as OperationalAuthority defines it.
  OperationalAuthority.AuthorityLevel authority_level_in_force = 7;

  // True only while this process holds a live command_authority lease, judged on its own clock.
  bool conn_held = 8;

  // Why the process is in this mode. For operators and logs; consumers MUST NOT parse it.
  string reason = 9;

  // Subjects this process needs and currently lacks or holds stale, e.g. "location_fix".
  repeated string degraded_inputs = 10;
}
```

## `messages/payloads/Encounter.proto`

```proto
// DRAFT for keelson/messages/payloads. Not yet proposed upstream.
//
// Subject:  encounter: keelson.Encounter
// Key:      {base_path}/@v0/{entity_id}/pubsub/encounter/{voyage_id | novoyage}/{encounter_id}
// QoS:      elevated
// Persist:  yes (one key per encounter, restated in full until it closes)
//
// Answers crowsnest-dev docs/NAVIGATION-DECISION-SERVICES.md §6.1. Until keelson registers it,
// keelson-processor-navigation publishes the same field set as a raw_json rider.

syntax = "proto3";

import "google/protobuf/timestamp.proto";
import "Route.proto";  // ColregSituation (the plan-side enum; extended below for a live encounter)

package keelson;

// One vessel-to-vessel encounter as own ship's navigation assesses it.
//
// A record, not a command, and not the advice: what own ship should DO about an encounter is a
// NavigationAdvice, which names the encounters it answers. Keeping them apart is what lets an
// assessment exist, and be recorded, whether or not anyone acts on it.
//
// EVERY RECORD IS A FULL RESTATEMENT, for the reason EnvelopeExceedance gives: the store keeps the
// last value per key, so a closing record must carry everything the opening one did.
message Encounter {

  // Own ship's role under the rules, from own ship's side.
  enum Role {
    ROLE_UNSPECIFIED = 0;
    ROLE_GIVE_WAY = 1;   // Rules 13, 14, 15, 16, 18: keep out of the way
    ROLE_STAND_ON = 2;   // Rule 17: keep course and speed
    ROLE_NONE = 3;       // Rule 19 (restricted visibility), or no rule applies
  }

  enum Risk {
    RISK_NONE = 0;
    RISK_WATCH = 1;
    RISK_RISK = 2;
    RISK_DANGER = 3;
  }

  // The live situation. ColregSituation's values, plus two the plan-side enum has no use for.
  // Carried as its own enum so the plan enum does not grow values a route cannot mean.
  enum Situation {
    COLREG_SITUATION_UNSPECIFIED = 0;
    COLREG_SITUATION_OVERTAKING = 1;     // own ship is overtaking
    COLREG_SITUATION_CROSSING_STBD = 2;  // target crossing from own starboard
    COLREG_SITUATION_CROSSING_PORT = 3;  // target crossing from own port
    COLREG_SITUATION_HEAD_ON = 4;
    COLREG_SITUATION_OVERTAKEN = 5;      // target is overtaking own ship
    COLREG_SITUATION_NONE = 6;           // target not under way, or restricted visibility
  }

  // Where the target was heard: the key it was published on, minus the subject.
  message TargetRef {
    string entity_id = 1;   // e.g. "maritimedb"
    string source_id = 2;   // e.g. "digitraffic"
    string target_id = 3;   // the @target token, e.g. "mmsi_265000000"
  }

  google.protobuf.Timestamp timestamp = 1;

  // UUID. Also the trailing key token, stable for the life of the encounter.
  string encounter_id = 2;

  // Absent when own ship has no voyage; the key then uses "novoyage".
  optional string voyage_id = 3;

  TargetRef target = 4;
  string target_name = 5;

  // At this record's timestamp.
  float range_m = 10;
  float bearing_true_deg = 11;
  float bearing_relative_deg = 12;
  optional float cpa_m = 13;
  optional float tcpa_s = 14;   // negative: closest approach already past

  // Latched when the encounter opened and held until it closes: the rules assign roles once,
  // and a role that flipped as the geometry evolved would contradict a manoeuvre already begun.
  Situation situation = 20;
  Role own_role = 21;

  // Rule references that decided situation and role, e.g. ["15", "16"], ["18(a)(iii)"].
  repeated string rules = 22;

  // Why, for operators and logs. Consumers MUST NOT parse it.
  string reasoning = 23;

  Risk risk = 30;
  Risk peak_risk = 31;

  google.protobuf.Timestamp opened_at = 40;
  // Absent while open, so an open encounter is distinguishable from one whose close was lost.
  optional google.protobuf.Timestamp closed_at = 41;

  // The operational context whose thresholds graded the risk, e.g. OPERATIONAL_CONTEXT_COASTAL.
  string threshold_context = 50;
}
```

## `messages/payloads/NavigationAdvice.proto`

```proto
// DRAFT for keelson/messages/payloads. Not yet proposed upstream.
//
// Subject:  navigation_advice: keelson.NavigationAdvice
// Key:      {base_path}/@v0/{entity_id}/pubsub/navigation_advice/{voyage_id | novoyage}/{advice_id}
// QoS:      elevated
// Persist:  yes
//
// Steering and propulsion mirror SteeringOrder / PropulsionOrder from keelson's
// feature/standing-order-verb (the frame is the oneof field). They are redeclared here because a
// payload cannot import an RPC interface's types; if those graduate to the payload pool, use them.

syntax = "proto3";

import "google/protobuf/timestamp.proto";

package keelson;

// What own ship's navigation advises doing about one or more encounters.
//
// ADVICE IS A RECORD, NEVER A COMMAND. Nothing acts on this message by receiving it. Acting on it
// is a separate, authorised publish on the vehicle interfaces, after an AdviceDisposition of
// ACCEPTED from whoever holds the decision. That separation is the point: the advisor can be the
// author of the advice without ever holding the conn.
message NavigationAdvice {

  enum Kind {
    KIND_UNSPECIFIED = 0;
    KIND_COURSE_ALTERATION = 1;
    KIND_SPEED_CHANGE = 2;
    KIND_COURSE_AND_SPEED = 3;
    KIND_STOP = 4;            // take all way off
    KIND_MAINTAIN = 5;        // stand on: keep course and speed (Rule 17(a)(i)), advised explicitly
    KIND_RESUME_TRACK = 6;    // the encounters this avoided have closed
  }

  message SteeringProposal {
    oneof frame {
      double course_over_ground_deg = 1;
      double heading_deg = 2;
    }
  }

  message PropulsionProposal {
    oneof frame {
      double sog_mps = 1;
      double stw_mps = 2;
    }
  }

  google.protobuf.Timestamp timestamp = 1;

  // UUID, and the trailing key token. Restated with the same id while the same manoeuvre holds;
  // a different manoeuvre is a new advice that supersedes this one.
  string advice_id = 2;
  optional string voyage_id = 3;

  Kind kind = 4;

  // The encounters this advice answers. Every open encounter with risk at or above RISK.
  repeated string encounter_ids = 5;

  // Absent when the kind does not change that half (a pure course alteration has no propulsion).
  SteeringProposal steering = 10;
  PropulsionProposal propulsion = 11;

  // The own-ship state the advice was computed from, so a reader can see the size of the change.
  optional double from_course_over_ground_deg = 12;
  optional double from_sog_mps = 13;

  // Consumers act within this window or not at all.
  google.protobuf.Timestamp valid_from = 20;
  google.protobuf.Timestamp valid_to = 21;

  // What the advisor predicts if the advice is followed now and held, against every target in
  // range, not only the ones in encounter_ids. Constant-velocity prediction for the targets.
  optional float expected_min_cpa_m = 30;
  optional float expected_min_range_m = 31;

  repeated string rules = 40;

  // Why, including what was NOT considered (e.g. "not checked against the chart"). Consumers MUST
  // NOT parse it.
  string rationale = 41;

  // Identifies the policy that produced this advice, as OperationalAuthority does.
  optional string policy_id = 50;
  optional bytes policy_config_digest = 51;
}
```

## `messages/payloads/AdviceDisposition.proto`

```proto
// DRAFT for keelson/messages/payloads. Not yet proposed upstream.
//
// Subject:  advice_disposition: keelson.AdviceDisposition
// Key:      {base_path}/@v0/{entity_id}/pubsub/advice_disposition/{voyage_id | novoyage}/{advice_id}
// QoS:      elevated
// Persist:  yes (last disposition per advice)

syntax = "proto3";

import "google/protobuf/timestamp.proto";

package keelson;

// What happened to one NavigationAdvice, and who decided.
//
// ACCEPTED and REJECTED come from whoever holds the decision for the function (an operator at a
// station, or a conduct service under a grant). SUPERSEDED and EXPIRED come from the advisor
// itself. Only ACCEPTED licenses acting, and only within the advice's validity window.
message AdviceDisposition {

  enum State {
    STATE_UNSPECIFIED = 0;
    STATE_ACCEPTED = 1;
    STATE_REJECTED = 2;
    STATE_SUPERSEDED = 3;
    STATE_EXPIRED = 4;
  }

  google.protobuf.Timestamp timestamp = 1;
  string advice_id = 2;
  optional string voyage_id = 3;
  State state = 4;

  // Who: "ted@ROC-1", or the source id of the service that decided.
  string decided_by = 5;
  string decided_site = 6;

  // For SUPERSEDED: the advice that replaced it.
  optional string superseded_by = 7;

  // Free text. Consumers MUST NOT parse it.
  string reason = 8;
}
```
