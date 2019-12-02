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
 *         MSF 6P COUNT handlers
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
#include "msf-housekeeping.h"
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
static void send_response(const linkaddr_t *peer_addr,
                          sixp_pkt_cell_options_t cell_options);

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
    LOG_ERR("COUNT transaction failed\n");
    /*
     * The peer may have lost cells; let's resolve the inconsistency.
     */
    msf_housekeeping_resolve_inconsistency(dest_addr);
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_callback_responder(void *arg, uint16_t arg_len,
                              const linkaddr_t *dest_addr,
                              sixp_output_status_t status)
{
  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    LOG_INFO("COUNT transaction completes successfully\n");
  } else {
    LOG_ERR("COUNT transaction failed\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
send_response(const linkaddr_t *peer_addr,
              sixp_pkt_cell_options_t cell_options)
{
  msf_negotiated_cell_type_t cell_type;
  uint16_t num_cells;
  sixp_pkt_rc_t rc;
  uint8_t body[sizeof(sixp_pkt_total_num_cells_t)];

  if(cell_options == SIXP_PKT_CELL_OPTION_TX) {
    cell_type = MSF_NEGOTIATED_CELL_TYPE_TX;
    rc = SIXP_PKT_RC_SUCCESS;
  } else if(cell_options == SIXP_PKT_CELL_OPTION_RX) {
    cell_type = MSF_NEGOTIATED_CELL_TYPE_RX;
    rc = SIXP_PKT_RC_SUCCESS;
  } else {
    LOG_ERR("Unsupported cell options: %u\n", cell_options);
    /*
     * Although we could response with RC_SUCCESS and NumCells of zero
     * according to RFC8480, RC_ERR will be returned to express the
     * given CellOptions value is not supported.
     */
    rc = SIXP_PKT_RC_ERR;
  }

  if(rc == SIXP_PKT_RC_SUCCESS) {
    num_cells = msf_negotiated_cell_get_num_cells(cell_type, peer_addr);
  } else {
    num_cells = 0;
  }

  if(sixp_pkt_set_total_num_cells(SIXP_PKT_TYPE_RESPONSE,
                                  (sixp_pkt_code_t)(uint8_t)rc, num_cells,
                                  body, sizeof(body)) < 0) {
    LOG_ERR("failed to build a response for COUNT\n");
  } else if(sixp_output(SIXP_PKT_TYPE_RESPONSE,
                        (sixp_pkt_code_t)(uint8_t)rc,
                        MSF_SFID, body, sizeof(body), peer_addr,
                        sent_callback_responder, NULL, 0) < 0) {
    LOG_ERR("failed to send a response to ");
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_("\n");
  } else {
    LOG_INFO("sent a response with %s to ", msf_sixp_get_rc_str(rc));
    LOG_INFO_LLADDR(peer_addr);
    LOG_INFO_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_count_send_request(void)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  uint8_t body[sizeof(sixp_pkt_metadata_t) +
               sizeof(sixp_pkt_cell_options_t)];

  memset(body, 0, sizeof(body));

  if(sixp_pkt_set_cell_options(SIXP_PKT_TYPE_REQUEST,
                               (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_COUNT,
                               SIXP_PKT_CELL_OPTION_TX,
                               body, sizeof(body)) < 0) {
    LOG_ERR("cannot build an COUNT request\n");
  } else

  if(parent_addr == NULL ||
     sixp_output(SIXP_PKT_TYPE_REQUEST,
                 (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_COUNT,
                 MSF_SFID, body, sizeof(body), parent_addr,
                 sent_callback_initiator, NULL, 0) < 0) {
    LOG_ERR("failed to send a COUNT request");
    if(parent_addr != NULL) {
      LOG_ERR(" to ");
      LOG_ERR_LLADDR(parent_addr);
    }
    LOG_ERR_("\n");
  } else {
    LOG_INFO("sent a COUNT request to ");
    LOG_INFO_LLADDR(parent_addr);
    LOG_INFO_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_count_recv_request(const linkaddr_t *peer_addr,
                            const uint8_t *body, uint16_t body_len)
{
  const sixp_pkt_type_t type = SIXP_PKT_TYPE_REQUEST;
  const sixp_pkt_code_t code = (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_ADD;
  sixp_pkt_cell_options_t cell_options;

  assert(peer_addr != NULL);
  LOG_INFO("received a COUNT request from ");
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(sixp_pkt_get_cell_options(type, code, &cell_options, body, body_len) < 0) {
    LOG_ERR("parse error\n");
  } else {
    send_response(peer_addr, cell_options);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_count_recv_response(const linkaddr_t *peer_addr, sixp_pkt_rc_t rc)
{
  /* we don't care about NumCells */
  LOG_INFO("received a COUNT response with %s from ", msf_sixp_get_rc_str(rc));
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(rc == SIXP_PKT_RC_SUCCESS) {
    LOG_INFO("COUNT transaction completes successfully\n");
  } else{
    LOG_ERR("COUNT transaction failed\n");
  }
}
/*---------------------------------------------------------------------------*/
