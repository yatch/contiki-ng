/* cannot use 'const' declaration in cooja's Javascript runtime */
var node_1 = sim.getMoteWithID(1)
var node_2 = sim.getMoteWithID(2)

/* this test lasts 10min (600s); GENERATE_MSG() takes a timeout value in ms */
GENERATE_MSG(600000, "test is done");

while(true) {
  YIELD();

  if(msg.equals("test is done")) {
    log.testFailed();
  }

  log.log(time + " node-" + id + " "+ msg + "\n");

  /* we don't expect any ERR from 6top or MSF in this scenario */
  if((msg.indexOf("[ERR : 6top       ]") !== -1 ||
      msg.indexOf("[ERR : MSF       ]") !== -1)) {
    log.testFailed();
  }

  if(id === 2) {
    if(msg.indexOf("COUNT transaction completes successfully") !== -1) {
      log.testOK();
    }
  }
}
