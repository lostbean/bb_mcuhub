# Architecture Decision Records

The recorded decisions behind `bb_mcuhub`, in order. Each one states the
problem, the options considered, and why the chosen shape won. The architecture
itself lives in [`docs/hub-design.html`](../hub-design.html) (open it locally
in a browser); the dated history is in
[`design_changelog.md`](../../design_changelog.md); the vocabulary is in
[`CONTEXT.md`](../../CONTEXT.md).

| ADR                                                          | Decision, in one phrase                                                                          |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------ |
| [0001](0001-can-segmentation-encoding.md)                    | CAN segmentation rides the reserved id bits, with an end-to-end CRC trailer                      |
| [0002](0002-single-source-dsl-spark-extension.md)            | The contract is authored in BeamBots' DSL; `bb_mcuhub` is a Spark extension                      |
| [0003](0003-library-example-split.md)                        | `bb_mcuhub` is a reusable library; `segby_v1` is a consumer example                              |
| [0004](0004-observer-plane.md)                               | Observability is a separate plane — observers sample the slots, decoupled from control           |
| [0005](0005-safe-action-is-a-value-type-value.md)            | A safe action is a value of the port's value-type; the floor is byte-generic                     |
| [0006](0006-links-are-declared-not-inferred.md)              | Topology is declared by parent links, not inferred; a link is a first-class edge                 |
| [0007](0007-one-writer-per-slot-is-a-capability.md)          | One writer per slot, enforced by a write-scoped capability (the mirror of the Reader)            |
| [0008](0008-virtual-robot-sim-in-the-loop.md)                | A robot can run virtually: a sim transport closes the host loop over a physics engine            |
| [0009](0009-wheel-velocity-sensor-and-host-velocity-loop.md) | A wheel reports measured speed as a sensor port; the host closes a velocity loop (example-only)  |
| [0010](0010-a-control-loop-falls-silent-on-disarm.md)        | A host control loop must fall silent on disarm — it produces the command-silence the floor needs |
| [0011](0011-router-is-direction-aware.md)                    | The router is direction-aware: a downlink arrival ascends; the route table serves only descents  |
