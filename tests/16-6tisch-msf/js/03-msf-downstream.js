/* cannot use 'const' declaration in cooja's Javascript runtime */
var INITIAL_PING_INTERVAL_MS = 500;
var NODE_2_IP_ADDR = "fd00::202:2:2:2";

var node_1 = sim.getMoteWithID(1)
var node_2 = sim.getMoteWithID(2)

var scheduled_ping = false;
var ping_interval_ms = INITIAL_PING_INTERVAL_MS;

/* this test lasts 10min (600s); GENERATE_MSG() takes a timeout value in ms */
GENERATE_MSG(600000, "test is done");

function ping_to_node_2() {
  write(node_1, "ping " + NODE_2_IP_ADDR + "\n");
  if(ping_interval_ms > 0) {
    GENERATE_MSG(ping_interval_ms, "send a ping");
  }
}

while(true) {
  YIELD();

  if(msg.equals("test is done")) {
    log.testFailed();
  } else if(msg.equals("send a ping")) {
    /* ping at every a half second */
    ping_to_node_2();
    continue;
  }

  log.log(time + " node-" + id + " "+ msg + "\n");

  /* we don't expect any ERR from 6top or MSF in this scenario */
  if((msg.indexOf("[ERR : 6top       ]") !== -1 ||
      msg.indexOf("[ERR : MSF       ]") !== -1)) {
    log.testFailed();
  }

  if(id === 2) {
    if(msg.indexOf("added a negotiated TX cell") !== -1 &&
       scheduled_ping == false) {
      GENERATE_MSG(500, "send a ping");
      scheduled_ping = true;
    } else if(msg.indexOf("added a negotiated RX cell") !== -1) {
      /* a negotiated RX cell is scheduled at node-2 as expected */
      /* incrase the pace */
      ping_interval_ms /= 2;
      if(ping_interval_ms < (INITIAL_PING_INTERVAL_MS / 4)) {
        /* stop pinging */
        ping_interval_ms = 0;
      }
    } else if(msg.indexOf("removed a negotiated RX cell") !== -1) {
      log.testOK();
    }
  }
}
