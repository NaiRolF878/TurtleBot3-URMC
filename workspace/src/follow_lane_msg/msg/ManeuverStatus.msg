# Stand einer Grundbewegung (/control/maneuver_status, von control_driving) bzw. eines Ablaufs
# aus Grundbewegungen (/control/maneuver_sequence, von switch_control).

uint32 id
# Nur Ablauf: laeuft noch.
bool active
# Beendet: erreicht (true) oder abgebrochen (false, Grund in reason).
bool success
string reason
