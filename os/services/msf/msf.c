/*
 * Copyright (c) 2019, Inria.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 *         MSF external APIs
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include <stdbool.h>
#include <string.h>

#include "contiki.h"
#include "lib/assert.h"

#include "net/linkaddr.h"
#include "net/mac/mac.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"
#include "services/shell/serial-shell.h"

#include "msf.h"
#include "msf-autonomous-cell.h"
#include "msf-housekeeping.h"
#include "msf-negotiated-cell.h"
#include "msf-reserved-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

typedef void sub_cmd(shell_output_func output);

/* static functions */
static void init(void);
static void input_handler(sixp_pkt_type_t type, sixp_pkt_code_t code,
                          const uint8_t *body, uint16_t body_len,
                          const linkaddr_t *src_addr);
static void timeout_handler(sixp_pkt_cmd_t cmd,
                            const linkaddr_t *peer_addr);
static void error_handler(sixp_error_t err, sixp_pkt_cmd_t cmd,
                          uint8_t seqno, const linkaddr_t *peer_addr);

/* variables */
static bool activated = false;

/*---------------------------------------------------------------------------*/
const struct MSFConstantInfo msf_constants_patern[] = {
  {
    "MSF Constants/Parameters:\n",  0
  },
  {
    "o SLOT_LENGTH                  : %ums\n",
    MSF_SLOT_LENGTH_MS
  },
  {
    "o NUM_CH_OFFSET                : %u\n",
    0
  },
  {
    "o MAX_NUM_CELLS                : %u\n",
    MSF_MAX_NUM_CELLS
  },
  {
    "o LIM_NUM_CELLS_USED_HIGH      : %u%%\n",
    MSF_LIM_NUM_CELLS_USED_HIGH
  },
  {
    "o LIM_NUM_CELLS_USED_LOW       : %u%%\n",
    MSF_LIM_NUM_CELLS_USED_LOW
  },
  {
    "o HOUSEKEEPING_COLLISION_PERIOD: %umin\n",
    MSF_HOUSEKEEPING_COLLISION_PERIOD_MIN
  },
  {
    "o HOUSEKEEPING_GC_PERIOD       : %umin\n",
    MSF_HOUSEKEEPING_GC_PERIOD_MIN,
  },
  {
    "o MSF_MIN_NUM_TX_FOR_RELOCATION: %u\n",
    MSF_MIN_NUM_TX_FOR_RELOCATION
  },
  {
    "o MSF_RELOCATE_PDR_THRESHOLD   : %u%%\n",
    MSF_RELOCATE_PDR_THRESHOLD
  },
  {
    "o WAIT_DURATION_MIN            : %us\n",
    MSF_WAIT_DURATION_MIN_SECONDS
  },
  {
    "o WAIT_DURATION_MAX            : %us\n",
    MSF_WAIT_DURATION_MAX_SECONDS
  },
  {
    "o MAX_NUM_NEGOTIATED_TX_CELLS  : %u\n",
    MSF_MAX_NUM_NEGOTIATED_TX_CELLS
  },
  {
    "o MAX_NUM_NEGOTIATED_RX_CELLS  : %u\n",
    MSF_MAX_NUM_NEGOTIATED_RX_CELLS
  },
  {
    NULL, 0
  },
};

void msf_constants_describe(MSFConstantInfo* info){
    memcpy(info, msf_constants_patern, MSFCONSTANTS_NUM* sizeof(MSFConstantInfo) );
    info[2].val = tsch_hopping_sequence_length.val;
}

static void
log_constants(){
    MSFConstantInfo constants[MSFCONSTANTS_NUM];
    msf_constants_describe(constants);

    for(int i = 0; constants[i].str != NULL; i++) {
        LOG_INFO(constants[i].str, constants[i].val);
    }
}

/*---------------------------------------------------------------------------*/
static void
init(void)
{
  static bool show_constants_now = true;
  if(show_constants_now) {
    log_constants();
    show_constants_now = false;
  } else {
    /* skip */
  }
}
/*---------------------------------------------------------------------------*/
static void
input_handler(sixp_pkt_type_t type, sixp_pkt_code_t code,
              const uint8_t *body, uint16_t body_len,
              const linkaddr_t *src_addr)
{
  sixp_trans_t *trans;
  sixp_pkt_cmd_t cmd;

  if(activated == false) {
    LOG_ERR("MSF is not activated; ignore the input 6P packet\n");
    return;
  } else if((trans = sixp_trans_find(src_addr)) == NULL) {
    LOG_ERR("cannot find a 6P transaction of a received 6P packet\n");
    return;
  } else {
    cmd = sixp_trans_get_cmd(trans);
  }

  if(type == SIXP_PKT_TYPE_REQUEST) {
    assert(code.value == cmd);
    if(code.value == SIXP_PKT_CMD_ADD) {
      msf_sixp_add_recv_request(src_addr, body, body_len);
    } else if(code.value == SIXP_PKT_CMD_DELETE) {
      msf_sixp_delete_recv_request(src_addr, body, body_len);
    } else if(code.value == SIXP_PKT_CMD_RELOCATE) {
      msf_sixp_relocate_recv_request(src_addr, body, body_len);
    } else if(code.value == SIXP_PKT_CMD_COUNT) {
      msf_sixp_count_recv_request(src_addr, body, body_len);
    } else if(code.value == SIXP_PKT_CMD_CLEAR) {
      msf_sixp_clear_recv_request(src_addr);
    }
  } else if(type == SIXP_PKT_TYPE_RESPONSE) {
    if(cmd == SIXP_PKT_CMD_ADD) {
      msf_sixp_add_recv_response(src_addr, (sixp_pkt_rc_t)code.value,
                            body, body_len);
    } else if(cmd == SIXP_PKT_CMD_DELETE) {
      msf_sixp_delete_recv_response(src_addr, (sixp_pkt_rc_t)code.value,
                               body, body_len);
    } else if(cmd == SIXP_PKT_CMD_RELOCATE) {
      msf_sixp_relocate_recv_response(src_addr,
                             (sixp_pkt_rc_t)code.value, body, body_len);
    } else if(cmd == SIXP_PKT_CMD_COUNT) {
      msf_sixp_count_recv_response(src_addr, (sixp_pkt_rc_t)code.value);
    } else if(cmd == SIXP_PKT_CMD_CLEAR) {
      msf_sixp_clear_recv_response(src_addr, (sixp_pkt_rc_t)code.value);
    }
  } else {
    /* MSF doesn't use 3-step transactions */
    LOG_ERR("received a 6P Confirmation, which is not supported by MSF\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(sixp_pkt_cmd_t cmd, const linkaddr_t *peer_addr)
{
  assert(peer_addr != NULL);
  if(cmd == SIXP_PKT_CMD_ADD) {
    LOG_INFO("ADD transaction ends because of timeout\n");
    msf_reserved_cell_delete_all(peer_addr);
  } else {
    /* do nothing */
  }

  if(linkaddr_cmp(peer_addr, msf_housekeeping_get_parent_addr())) {
    /* we are the initiator */
  } else {
    /* we are the responder */
    /*
     * schedule inconsistency may happen because of this timeout of
     * the transaction, where the peer completes the transaction by
     * our L2 MAC, but we don't. Better to confirm if there is
     * schedule consistency.
     */
  }
}
/*---------------------------------------------------------------------------*/
static void
error_handler(sixp_error_t err, sixp_pkt_cmd_t cmd, uint8_t seqno,
              const linkaddr_t *peer_addr)
{
  LOG_WARN("A 6P transaction for (cmd: %u) with ", cmd);
  LOG_WARN_LLADDR(peer_addr);
  LOG_WARN_(" ends with an error (err: %u)\n", err);
  if(err == SIXP_ERROR_SCHEDULE_INCONSISTENCY) {
    msf_negotiated_cell_delete_all(peer_addr);
  }
}
/*---------------------------------------------------------------------------*/
bool
msf_is_activated(void)
{
  return activated;
}
/*---------------------------------------------------------------------------*/
bool
msf_is_ready(void)
{
  bool ret = false;

  if(activated) {
    if(tsch_is_coordinator) {
      ret = true;
    } else {
      const uip_ipaddr_t *defrt;
      const linkaddr_t *parent_addr;
      tsch_neighbor_t *nbr;

      defrt = uip_ds6_defrt_choose();

      if(defrt == NULL) {
        parent_addr = NULL;
      } else {
        parent_addr = (const linkaddr_t *)uip_ds6_nbr_lladdr_from_ipaddr(defrt);
      }

      if(parent_addr == NULL) {
        ret = false;
      } else {
        nbr = tsch_queue_get_nbr(parent_addr);
        ret = msf_negotiated_cell_is_scheduled_tx(nbr);
      }
    }
  } else {
    ret = false;
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
void
msf_activate(void)
{
  assert(activated == false);

  if(tsch_schedule_get_slotframe_by_handle(
       MSF_SLOTFRAME_HANDLE_AUTONOMOUS_CELLS) == NULL &&
     tsch_schedule_add_slotframe(MSF_SLOTFRAME_HANDLE_AUTONOMOUS_CELLS,
                                 MSF_SLOTFRAME_LENGTH) == NULL) {
    LOG_ERR("failed to add a slotframe for the autonomous cells\n");
  }
  if(tsch_schedule_get_slotframe_by_handle(
       MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS) == NULL &&
     tsch_schedule_add_slotframe(MSF_SLOTFRAME_HANDLE_NEGOTIATED_CELLS,
                                 MSF_SLOTFRAME_LENGTH) == NULL) {
    LOG_ERR("failed to add a slotframe for the negotiated cells\n");
  }

  if(msf_autonomous_cell_activate() < 0) {
    LOG_ERR("cannot add the autonomous RX cell; failed to activate MSF\n");
  } else {
    msf_negotiated_cell_activate();
    activated = true;
    /* start the housekeeping process */
    msf_housekeeping_start();
    LOG_INFO("MSF is activated\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_deactivate(void)
{
  if(activated == true) {
    /* XXX: it would be better to abort all on-going 6P transactions */

    /* remove all the autonomous/negotiated/reserved cells */
    msf_autonomous_cell_deactivate();
    msf_negotiated_cell_deactivate();

    activated = false;
    /* stop the housekeeping */
    msf_housekeeping_stop();
    LOG_INFO("MSF is deactivated\n");
  }
}
/*---------------------------------------------------------------------------*/
const sixtop_sf_t msf = {
  MSF_SFID,
  (((2 << (TSCH_MAC_MAX_BE - 1)) - 1) *
   TSCH_MAC_MAX_FRAME_RETRIES *
   MSF_SLOTFRAME_LENGTH * MSF_SLOT_LENGTH_MS / 1000 * CLOCK_SECOND),
  init,
  input_handler,
  timeout_handler,
  error_handler,
};
/*---------------------------------------------------------------------------*/
