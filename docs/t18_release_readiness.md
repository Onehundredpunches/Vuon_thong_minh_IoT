# T-18 Release Readiness Summary

## Passed scope

- Simulator and hardware builds pass.
- Hardware upload on COM4 passes.
- Boot self-test covers pin map, actuator safety, sensor policy, command parser, mode FSM, auto logic, MQTT reconnect, MQTT command ACK, retained MQTT state, full integration, and LCD formatter.
- Hardware MQTT demo path validates Wi-Fi connection, MQTT connection, command ACK flow, retained state/status publishing, reconnect, and local DATA continuity.

## Remaining risks

- Public HiveMQ demo broker is not an ownership-boundary proof. Owner-managed broker ACL/auth/clientId/QoS/retain policy still requires project-owner validation.
- PubSubClient publish calls are retained where required, but broker-level QoS enforcement remains owner-managed.
- Long soak test target remains 12-24 hours; this run verifies functional readiness, not long-duration stability.
- Physical actuator feedback is open-loop; ACK means firmware accepted and invoked the actuator path, not hardware closed-loop confirmation.
