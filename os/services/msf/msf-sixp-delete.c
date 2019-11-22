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
 *         MSF 6P DELETE handlers
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
static bool is_valid_request(sixp_pkt_cell_options_t cell_options,
                             sixp_pkt_num_cells_t num_cells,
                             const uint8_t *cell_list, uint16_t cell_list_len);
static void sent_callback_initiator(void *arg, uint16_t arg_len,
                                    const linkaddr_t *dest_addr,
                                    sixp_output_status_t status);
static void sent_callback_responder(void *arg, uint16_t arg_len,
                                    const linkaddr_t *dest_addr,
                                    sixp_output_status_t status);
static void send_response(const linkaddr_t *peer_addr,
                          sixp_pkt_cell_options_t cell_options,
                          sixp_pkt_num_cells_t num_cells,
                          const uint8_t *cell_list,
                          uint16_t cell_list_len);

/*---------------------------------------------------------------------------*/
static bool
is_valid_request(sixp_pkt_cell_options_t cell_options,
                 sixp_pkt_num_cells_t num_cells,
                 const uint8_t *cell_list, uint16_t cell_list_len)
{
  bool ret = false;

  if(cell_options != SIXP_PKT_CELL_OPTION_TX &&
     cell_options != SIXP_PKT_CELL_OPTION_RX) {
    LOG_INFO("bad CellOptions - %02X (should be %02X or %02X)\n",
             cell_options, SIXP_PKT_CELL_OPTION_TX, SIXP_PKT_CELL_OPTION_RX);
  } else if(num_cells != 1) {
    LOG_INFO("bad NumCells - %u (should be 1)\n", num_cells);
  } else if(cell_list == NULL) {
    LOG_INFO("no CellList\n");
  } else if(cell_list_len != sizeof(sixp_pkt_cell_t)) {
    LOG_INFO("too short CellList - %u octets\n", cell_list_len);
  } else {
    ret = true;
  }
  return ret;
}
/*---------------------------------------------------------------------------*/
static void
sent_callback_initiator(void *arg, uint16_t arg_len,
                        const linkaddr_t *dest_addr,
                        sixp_output_status_t status)
{
  tsch_link_t *cell_to_delete = (tsch_link_t *)arg;
  if(cell_to_delete == NULL) {
    /* do nothing */
  } else {
    if(status != SIXP_OUTPUT_STATUS_SUCCESS &&
       cell_to_delete->link_options == LINK_OPTION_TX) {
      /*
       * The peer may have gone; delete the TX cell. Even if the
       * failure occurs just by bad luck, it should be fine. A
       * possible schedule inconsistency should be detected during a
       * following transaction if any.
       */
      msf_negotiated_cell_delete(cell_to_delete);
    } else {
      /* do nothing */
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_callback_responder(void *arg, uint16_t arg_len,
                        const linkaddr_t *dest_addr,
                        sixp_output_status_t status)
{
  tsch_link_t *cell_to_delete = (tsch_link_t *)arg;

  /* we are the responder of the DELETE transaction */
  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    LOG_INFO("DELETE transaction completes successfully\n");
    if(cell_to_delete != NULL) {
      msf_negotiated_cell_delete(cell_to_delete);
    } else {
      /* all good */
    }
  } else {
    /* schedule inconsistency may happen; we don't do anything */
    LOG_ERR("DELETE transaction failed\n");
  }
}
/*---------------------------------------------------------------------------*/
static void
send_response(const linkaddr_t *peer_addr,
              sixp_pkt_cell_options_t cell_options,
              sixp_pkt_num_cells_t num_cells,
              const uint8_t *cell_list, uint16_t cell_list_len)
{
  sixp_pkt_rc_t rc;
  tsch_link_t *cell_to_delete;

  assert(peer_addr != NULL);

  if(is_valid_request(cell_options, num_cells,
                      cell_list, cell_list_len) == false) {
    /* invalid value for DELETE request of MSF */
    rc = SIXP_PKT_RC_ERR;
    cell_to_delete = NULL;
  } else if((cell_to_delete =
             msf_sixp_find_scheduled_cell(peer_addr,
                                          LINK_OPTION_RX,
                                          cell_list,
                                          cell_list_len)) == NULL) {
    rc = SIXP_PKT_RC_ERR_CELLLIST;
  } else {
    rc = SIXP_PKT_RC_SUCCESS;
  }

  if(sixp_output(SIXP_PKT_TYPE_RESPONSE, (sixp_pkt_code_t)(uint8_t)rc,
                 MSF_SFID,
                 rc == SIXP_PKT_RC_SUCCESS ? (const uint8_t *)cell_list : NULL,
                 rc == SIXP_PKT_RC_SUCCESS ? sizeof(sixp_pkt_cell_t) : 0,
                 peer_addr,
                 sent_callback_responder,
                 cell_to_delete, sizeof(tsch_link_t)) < 0) {
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
msf_sixp_delete_send_request(msf_negotiated_cell_type_t cell_type)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  const sixp_pkt_type_t type = SIXP_PKT_TYPE_REQUEST;
  const sixp_pkt_code_t code = (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_DELETE;
  tsch_neighbor_t *nbr;
  const size_t cell_list_len = sizeof(sixp_pkt_cell_t);
  const size_t body_len = (sizeof(sixp_pkt_metadata_t) +
                           sizeof(sixp_pkt_cell_options_t) +
                           sizeof(sixp_pkt_num_cells_t) +
                           cell_list_len);
  uint8_t cell_list[sizeof(sixp_pkt_cell_t)];
  uint8_t body[body_len];
  sixp_pkt_cell_options_t cell_options;
  const sixp_pkt_num_cells_t num_cells = 1;
  tsch_link_t *cell_to_delete;

  assert(parent_addr != NULL);

  nbr = tsch_queue_get_nbr(parent_addr);
  if(nbr == NULL ||
     (cell_to_delete =
      msf_negotiated_cell_get_cell_to_delete(parent_addr, cell_type)) == NULL) {
    /* this shouldn't happen, by the way */
    LOG_ERR("delete_send_request: no negotiated %s cells scheduled with ",
            cell_type == MSF_NEGOTIATED_CELL_TYPE_TX ? "TX" : "RX");
    LOG_ERR_LLADDR(parent_addr);
    LOG_ERR_("\n");
  } else {
    if(cell_type == MSF_NEGOTIATED_CELL_TYPE_TX) {
      cell_options = SIXP_PKT_CELL_OPTION_TX;
    } else {
      cell_options = SIXP_PKT_CELL_OPTION_RX;
    }
    msf_sixp_set_cell_params(cell_list, cell_to_delete);

    /* build a body of DELETE request */
    memset(body, 0, body_len);
    if(sixp_pkt_set_cell_options(type, code,
                                 cell_options, body, body_len) < 0 ||
       sixp_pkt_set_num_cells(type, code, num_cells, body, body_len) < 0 ||
       sixp_pkt_set_cell_list(type, code, cell_list, cell_list_len,
                              0, body, body_len) < 0) {
      LOG_ERR("cannot build a DELETE request\n");
      msf_sixp_start_request_wait_timer();
    } else if(sixp_output(type, code, MSF_SFID, body, body_len, parent_addr,
                          sent_callback_initiator,
                          cell_to_delete, sizeof(cell_to_delete)) < 0) {
      LOG_ERR("failed to send a DELETE request to \n");
      LOG_ERR_LLADDR(parent_addr);
      LOG_ERR_("\n");
      msf_sixp_start_request_wait_timer();
    } else {
      LOG_INFO("sent a DELETE request to the parent: ");
      LOG_INFO_LLADDR(parent_addr);
      LOG_INFO_("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_delete_recv_request(const linkaddr_t *peer_addr,
                             const uint8_t *body, uint16_t body_len)
{
  const sixp_pkt_type_t type = SIXP_PKT_TYPE_REQUEST;
  const sixp_pkt_code_t code = (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_DELETE;
  sixp_pkt_cell_options_t cell_options;
  sixp_pkt_num_cells_t num_cells;
  const uint8_t *cell_list;
  uint16_t cell_list_len;

  assert(peer_addr != NULL);

  LOG_INFO("received a DELETE request from ");
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(sixp_pkt_get_cell_options(type, code, &cell_options, body, body_len) < 0 ||
     sixp_pkt_get_num_cells(type, code, &num_cells, body, body_len) < 0 ||
     sixp_pkt_get_cell_list(type, code, &cell_list, &cell_list_len,
                            body, body_len) < 0) {
    LOG_ERR("parse error\n");
  } else {
    send_response(peer_addr, cell_options, num_cells, cell_list, cell_list_len);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_delete_recv_response(const linkaddr_t *peer_addr, sixp_pkt_rc_t rc,
                              const uint8_t *body, uint16_t body_len)
{
  const uint8_t *cell_list;
  uint16_t cell_list_len;
  tsch_link_t *cell;

  assert(peer_addr != NULL);

  LOG_INFO("received an DELETE response with %s from ",
           msf_sixp_get_rc_str(rc));
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(rc == SIXP_PKT_RC_SUCCESS) {
    LOG_INFO("DELETE transaction completes successfully\n");
    if(sixp_pkt_get_cell_list(SIXP_PKT_TYPE_RESPONSE,
                              (sixp_pkt_code_t)(uint8_t)SIXP_PKT_RC_SUCCESS,
                              &cell_list, &cell_list_len,
                              body, body_len) < 0) {
      LOG_ERR("parse error\n");
    } else if(cell_list_len != sizeof(sixp_pkt_cell_t)) {
      LOG_ERR("received an invalid CellList (%u octets)\n", cell_list_len);
    } else if((cell = msf_sixp_find_scheduled_cell(peer_addr,
                                                   LINK_OPTION_TX,
                                                   cell_list,
                                                   cell_list_len)) == NULL) {
      msf_housekeeping_resolve_inconsistency(peer_addr);
    } else {
      /* all good */
      msf_negotiated_cell_delete(cell);
    }
  } else if(rc == SIXP_PKT_RC_ERR_CELLLIST ||
            rc == SIXP_PKT_RC_ERR_SEQNUM) {
    msf_housekeeping_resolve_inconsistency(peer_addr);
  } else {
    /* do nothing */
  }
}
/*---------------------------------------------------------------------------*/
