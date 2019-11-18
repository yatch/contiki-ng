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
 *         MSF 6P CLEAR handlers
 * \author
 *         Yasuyuki Tanaka <yasuyuki.tanaka@inria.fr>
 */

#include "contiki.h"
#include "assert.h"

#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/sixtop/sixtop.h"
#include "net/mac/tsch/sixtop/sixp.h"
#include "net/mac/tsch/sixtop/sixp-trans.h"

#include "msf.h"
#include "msf-negotiated-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* static functions */
static void sent_callback_initiator(void *arg, uint16_t arg_len,
                                          const linkaddr_t *dest_addr,
                                          sixp_output_status_t status);
static void sent_callback_responder(void *arg, uint16_t arg_len,
                                          const linkaddr_t *dest_addr,
                                          sixp_output_status_t status);
static void send_response(const linkaddr_t *peer_addr);

/*---------------------------------------------------------------------------*/
static void
sent_callback_initiator(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status)
{
  /* this callback is expected to be set only for request */
  /*
   * in any case, remove all the negotiated TX cells without waiting
   * for a response
   */
  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    /* do nothing */
  } else {
    LOG_ERR("CLEAR transaction failed\n");
    msf_negotiated_cell_delete_all(dest_addr);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_callback_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status)
{
  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    LOG_INFO("CLEAR transaction completes successfully\n");
  } else {
    LOG_ERR("CLEAR transaction failed\n");
  }
  msf_negotiated_cell_delete_all(dest_addr);
}
/*---------------------------------------------------------------------------*/
static void
send_response(const linkaddr_t *peer_addr)
{
  if(sixp_output(SIXP_PKT_TYPE_RESPONSE,
                 (sixp_pkt_code_t)(uint8_t)SIXP_PKT_RC_SUCCESS,
                 MSF_SFID, NULL, 0, peer_addr,
                 sent_callback_responder, NULL, 0) < 0) {
    LOG_ERR("failed to send a response to ");
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_("\n");
  } else {
    LOG_DBG("sent a response with RC_SUCCESS to ");
    LOG_DBG_LLADDR(peer_addr);
    LOG_DBG_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_clear_send_request(const linkaddr_t *peer_addr)
{
  uint8_t body[sizeof(sixp_pkt_metadata_t)];
  memset(body, 0, sizeof(body));

  assert(peer_addr != NULL);
  if(sixp_output(SIXP_PKT_TYPE_REQUEST,
                 (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_CLEAR,
                 MSF_SFID, body, sizeof(body), peer_addr,
                 sent_callback_initiator, NULL, 0) < 0) {
    LOG_ERR("failed to send a CLEAR request to ");
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_("\n");
    /* remove all the negotiated TX cells without retry */
    msf_negotiated_cell_delete_all(peer_addr);
  } else {
    LOG_INFO("sent a CLEAR request to ");
    LOG_INFO_LLADDR(peer_addr);
    LOG_INFO_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_clear_recv_request(const linkaddr_t *peer_addr)
{
  assert(peer_addr != NULL);
  LOG_INFO("received a CLEAR request from ");
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");
  send_response(peer_addr);
  msf_negotiated_cell_delete_all(peer_addr);
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_clear_recv_response(const linkaddr_t *peer_addr, sixp_pkt_rc_t rc)
{
  LOG_INFO("received a CLEAR response with %s from ", msf_sixp_get_rc_str(rc));
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(rc == SIXP_PKT_RC_SUCCESS) {
    LOG_INFO("CLEAR transaction completes successfully\n");
  } else{
    LOG_ERR("CLEAR transaction failed\n");
  }
}
/*---------------------------------------------------------------------------*/
