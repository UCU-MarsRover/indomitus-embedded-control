#include <Arduino.h>
#include "can_comm.hpp"

#ifndef NODE_ROLE
#define NODE_ROLE 1
#endif

#ifndef NODE_ID
#define NODE_ID 1
#endif

static constexpr uint32_t REQ_ID = 0x120;
static constexpr uint32_t RESP_ID = 0x121;

#if NODE_ROLE == 1

static void receive_two_responses(uint8_t expected_seq, uint32_t window_ms = 500) {
  const uint32_t start = millis();
  bool got2 = false;
  bool got3 = false;
  uint8_t sum2 = 0;
  uint8_t sum3 = 0;

  while (millis() - start < window_ms) {
    CanMsg msg;
    if (!can_recv(msg, 20)) {
      continue;
    }

    if (msg.id != RESP_ID || msg.len < 3) {
      continue;
    }

    const uint8_t src = msg.data[0];
    const uint8_t sum = msg.data[1];
    const uint8_t seq = msg.data[2];

    if (seq != expected_seq) {
      continue;
    }

    if (src == 2) {
      got2 = true;
      sum2 = sum;
    } else if (src == 3) {
      got3 = true;
      sum3 = sum;
    }

    if (got2 && got3) {
      break;
    }
  }

  if (got2) {
    Serial.print("Got sum from node2 = ");
    Serial.println(sum2);
  } else {
    Serial.println("Node2 response missing (timeout)");
  }

  if (got3) {
    Serial.print("Got sum from node3 = ");
    Serial.println(sum3);
  } else {
    Serial.println("Node3 response missing (timeout)");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("NODE1 boot");
  can_init_500k_accept_all();
  Serial.println("CAN started");
}

void loop() {
  static uint8_t seq = 0;
  seq++;

  const uint8_t payload2[4] = {2, 10, 7, seq};
  const uint8_t payload3[4] = {3, 15, 6, seq};

  const bool ok2 = can_send(REQ_ID, payload2, 4);
  const bool ok3 = can_send(REQ_ID, payload3, 4);

  Serial.print("Send to node2 -> ");
  Serial.println(ok2 ? "OK" : "FAIL");
  Serial.print("Send to node3 -> ");
  Serial.println(ok3 ? "OK" : "FAIL");

  receive_two_responses(seq, 500);
  delay(2000);
}

#else

static bool recv_add_request_for_me(uint8_t my_id,
                                    uint8_t &a_out,
                                    uint8_t &b_out,
                                    uint8_t &seq_out,
                                    uint32_t timeout_ms = 1000) {
  CanMsg msg;
  if (!can_recv(msg, timeout_ms)) {
    return false;
  }

  if (msg.id != REQ_ID || msg.len < 4) {
    return false;
  }

  const uint8_t dest = msg.data[0];
  if (dest != my_id) {
    return false;
  }

  a_out = msg.data[1];
  b_out = msg.data[2];
  seq_out = msg.data[3];

  Serial.print("RX REQ for me. a=");
  Serial.print(a_out);
  Serial.print(" b=");
  Serial.print(b_out);
  Serial.print(" seq=");
  Serial.println(seq_out);

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.print("NODE");
  Serial.print(NODE_ID);
  Serial.println(" boot");
  can_init_500k_accept_all();
  Serial.println("CAN started");
}

void loop() {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t seq = 0;

  if (!recv_add_request_for_me(NODE_ID, a, b, seq, 1000)) {
    return;
  }

  const uint8_t sum = static_cast<uint8_t>(a + b);
  const uint8_t payload[3] = {NODE_ID, sum, seq};
  const bool ok = can_send(RESP_ID, payload, 3);

  Serial.print("TX RESP src=");
  Serial.print(NODE_ID);
  Serial.print(" sum=");
  Serial.print(sum);
  Serial.print(" seq=");
  Serial.print(seq);
  Serial.print(" -> ");
  Serial.println(ok ? "OK" : "FAIL");
}

#endif
