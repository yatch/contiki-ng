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
 *         MSF 6P RELOCATE handlers
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
#include "msf-reserved-cell.h"
#include "msf-sixp.h"

#include "sys/log.h"
#define LOG_MODULE "MSF"
#define LOG_LEVEL LOG_LEVEL_MSF

/* static functions */
static bool is_valid_request(sixp_pkt_cell_options_t cell_options,
                             sixp_pkt_num_cells_t num_cells,
                             const uint8_t *relocation_cell_list,
                             uint16_t relocation_cell_list_len,
                             const uint8_t *candidate_cell_list,
                             uint16_t candidate_cell_list_len);
static void sent_callback_initiator(void *arg, uint16_t arg_len,
                                             const linkaddr_t *dest_addr,
                                             sixp_output_status_t status);
static void sent_callback_responder(void *arg, uint16_t arg_len,
                                             const linkaddr_t *dest_addr,
                                             sixp_output_status_t status);
static void send_response(const linkaddr_t *peer_addr,
                                   sixp_pkt_cell_options_t cell_options,
                                   sixp_pkt_num_cells_t num_cells,
                                   const uint8_t *relocation_cell_list,
                                   uint16_t relocation_cell_list_len,
                                   const uint8_t *candidate_cell_list,
                                   uint16_t candidate_cell_list_len);

/*---------------------------------------------------------------------------*/
static bool
is_valid_request(sixp_pkt_cell_options_t cell_options,
                 sixp_pkt_num_cells_t num_cells,
                 const uint8_t *relocation_cell_list,
                 uint16_t relocation_cell_list_len,
                 const uint8_t *candidate_cell_list,
                 uint16_t candidate_cell_list_len)
{
  bool ret = false;

  if(cell_options != SIXP_PKT_CELL_OPTION_TX) {
    LOG_INFO("bad CellOptions - %02X (should be %02X)\n",
             cell_options, SIXP_PKT_CELL_OPTION_TX);
  } else if(num_cells != 1) {
    LOG_INFO("bad NumCells - %u (should be 1)\n", num_cells);
  } else if(relocation_cell_list == NULL) {
    LOG_INFO("no RelocationCellList\n");
  } else if(relocation_cell_list_len != sizeof(sixp_pkt_cell_t)) {
    LOG_INFO("bad RelocationCellList - %u octets\n",
             relocation_cell_list_len);
  } else if(candidate_cell_list == NULL) {
    LOG_INFO("no CandidateCellList\n");
  } else if(candidate_cell_list_len < sizeof(sixp_pkt_cell_t)) {
    LOG_INFO("too short CandidateCellList - %u octets\n",
             candidate_cell_list_len);
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
  assert(dest_addr != NULL);

  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    /* our request is acknowledged nothing to do */
  } else {
    assert(status == SIXP_OUTPUT_STATUS_FAILURE ||
           status == SIXP_OUTPUT_STATUS_ABORTED);
    LOG_ERR("RELOCATE transaction failed\n");
    msf_reserved_cell_delete_all(dest_addr);
    /* retry later */
    msf_sixp_start_request_wait_timer();
  }
}
/*---------------------------------------------------------------------------*/
static void
sent_callback_responder(void *arg, uint16_t arg_len,
                                 const linkaddr_t *dest_addr,
                                 sixp_output_status_t status)
{
  tsch_link_t *cell_to_relocate = (tsch_link_t *)arg;

  assert(dest_addr != NULL);
  assert(arg == NULL || arg_len == sizeof(tsch_link_t));

  if(status == SIXP_OUTPUT_STATUS_SUCCESS) {
    LOG_INFO("RELOCATE transaction completes successfully\n");
    if(cell_to_relocate == NULL) {
      /* we've returned an empty CellList; do nothing */
      assert(arg_len == 0);
    } else if(msf_reserved_cell_get_num_cells(dest_addr) != 1) {
      LOG_ERR("We have more than one reserved cell; this shouldn't happen\n");
      msf_reserved_cell_delete_all(dest_addr);
    } else {
      tsch_link_t *reserved_cell = msf_reserved_cell_get(dest_addr, -1, -1);
      uint16_t slot_offset, channel_offset;

      assert(reserved_cell != NULL);
      slot_offset = reserved_cell->timeslot;
      channel_offset = reserved_cell->channel_offset;
      msf_reserved_cell_delete_all(dest_addr);

      /* RELOCATE can happen only with negotiated RX cells */
      if(msf_negotiated_cell_add(dest_addr, MSF_NEGOTIATED_CELL_TYPE_RX,
                                 slot_offset, channel_offset) < 0) {
        /* this allocation failure could cause schedule inconsistency */
        LOG_ERR("failed to relocate\n");
        /* don't try to resolve the inconsitency from the responder */
      } else {
        msf_negotiated_cell_delete(cell_to_relocate);
        /* all good */
      }
    }
  } else {
    assert(status == SIXP_OUTPUT_STATUS_FAILURE ||
           status == SIXP_OUTPUT_STATUS_ABORTED);
    LOG_ERR("RELOCATE transaction failed\n");
    msf_reserved_cell_delete_all(dest_addr);
  }
}
/*---------------------------------------------------------------------------*/
static void
send_response(const linkaddr_t *peer_addr,
                       sixp_pkt_cell_options_t cell_options,
                       sixp_pkt_num_cells_t num_cells,
                       const uint8_t *relocation_cell_list,
                       uint16_t relocation_cell_list_len,
                       const uint8_t *candidate_cell_list,
                       uint16_t candidate_cell_list_len)
{
  tsch_link_t *reserved_cell;
  tsch_link_t *cell_to_relocate;
  sixp_pkt_cell_t cell_to_return;
  sixp_pkt_rc_t rc;

  assert(peer_addr != NULL);

  if(is_valid_request(cell_options, num_cells,
                      relocation_cell_list, relocation_cell_list_len,
                      candidate_cell_list, candidate_cell_list_len) == false) {
    rc = SIXP_PKT_RC_ERR;
    reserved_cell = NULL;
    cell_to_relocate = NULL;
  } else if((cell_to_relocate =
             msf_sixp_find_scheduled_cell(peer_addr,
                                          LINK_OPTION_RX,
                                          relocation_cell_list,
                                          relocation_cell_list_len)) == NULL) {
    rc = SIXP_PKT_RC_ERR_CELLLIST;
    reserved_cell = NULL;
  } else {
    rc = SIXP_PKT_RC_SUCCESS;
    /* RELOCATE can happen only with negotiated RX cells */
    if((reserved_cell =
             msf_sixp_reserve_one_cell(peer_addr,
                                       MSF_NEGOTIATED_CELL_TYPE_RX,
                                       candidate_cell_list,
                                       candidate_cell_list_len)) == NULL) {
      LOG_ERR("cannot reserve a cell; going to send an empty CellList\n");
    } else {
      msf_sixp_set_cell_params((uint8_t *)&cell_to_return, reserved_cell);
    }
  }

  if(sixp_output(SIXP_PKT_TYPE_RESPONSE, (sixp_pkt_code_t)(uint8_t)rc,
                 MSF_SFID,
                 reserved_cell == NULL ? NULL : (uint8_t *)&cell_to_return,
                 reserved_cell == NULL ? 0 : sizeof(sixp_pkt_cell_t),
                 peer_addr, sent_callback_responder, cell_to_relocate,
                 reserved_cell == NULL ? 0 : sizeof(tsch_link_t)) < 0) {
    LOG_ERR("failed to send a response to ");
    LOG_ERR_LLADDR(peer_addr);
    LOG_ERR_("\n");
    msf_reserved_cell_delete_all(peer_addr);
  } else {
    LOG_INFO("sent a response with %s to ", msf_sixp_get_rc_str(rc));
    LOG_INFO_LLADDR(peer_addr);
    LOG_INFO_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_relocate_send_request(const tsch_link_t *cell_to_relocate)
{
  const linkaddr_t *parent_addr = msf_housekeeping_get_parent_addr();
  const sixp_pkt_type_t type = SIXP_PKT_TYPE_REQUEST;
  const sixp_pkt_code_t code = (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_RELOCATE;
  const size_t relocation_cell_list_len = sizeof(sixp_pkt_cell_t);
  uint8_t relocation_cell_list[sizeof(sixp_pkt_cell_t)];
  uint8_t candidate_cell_list[(MSF_6P_CELL_LIST_MAX_LEN *
                               sizeof(sixp_pkt_cell_t))];
  size_t candidate_cell_list_len;
  uint8_t body[sizeof(sixp_pkt_metadata_t) +
               sizeof(sixp_pkt_cell_options_t) +
               sizeof(sixp_pkt_num_cells_t) +
               relocation_cell_list_len +
               sizeof(candidate_cell_list)];
  size_t body_len;
  sixp_pkt_cell_options_t cell_options = SIXP_PKT_CELL_OPTION_TX;
  const sixp_pkt_num_cells_t num_cells = 1;

  assert(parent_addr != NULL);
  assert(cell_to_relocate != NULL);

  msf_sixp_set_cell_params(relocation_cell_list, cell_to_relocate);
  /* RELOCATION can happen only with negotiated TX cells */
  candidate_cell_list_len = msf_sixp_fill_cell_list(
    parent_addr, MSF_NEGOTIATED_CELL_TYPE_TX,
    candidate_cell_list, sizeof(candidate_cell_list));

  memset(body, 0, sizeof(body));
  body_len = (sizeof(sixp_pkt_metadata_t) +
              sizeof(sixp_pkt_cell_options_t) +
              sizeof(sixp_pkt_num_cells_t) +
              relocation_cell_list_len +
              candidate_cell_list_len);

  if(candidate_cell_list_len == 0) {
    LOG_ERR("relocate_send_request: no cell is available\n");
    msf_sixp_start_request_wait_timer();
  } else if(
    sixp_pkt_set_cell_options(type, code, cell_options, body, body_len) < 0 ||
    sixp_pkt_set_num_cells(type, code, num_cells, body, body_len) < 0 ||
    sixp_pkt_set_rel_cell_list(type, code,
                               relocation_cell_list,
                               relocation_cell_list_len, 0,
                               body, body_len) < 0 ||
    sixp_pkt_set_cand_cell_list(type, code,
                                candidate_cell_list, candidate_cell_list_len,
                                0, body, body_len) < 0) {
    LOG_ERR("cannot build a RELOCATE request\n");
    msf_reserved_cell_delete_all(parent_addr);
    msf_sixp_start_request_wait_timer();
  } else if(sixp_output(type, code, MSF_SFID, body, body_len, parent_addr,
                        sent_callback_initiator, NULL, 0) < 0) {
    LOG_ERR("failed to send a RELOCATE request to \n");
    LOG_ERR_LLADDR(parent_addr);
    LOG_ERR_("\n");
    msf_reserved_cell_delete_all(parent_addr);
    msf_sixp_start_request_wait_timer();
  } else {
    LOG_INFO("sent a RELOCATE request to the parent: ");
    LOG_INFO_LLADDR(parent_addr);
    LOG_INFO_("\n");
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_relocate_recv_request(const linkaddr_t *peer_addr,
                               const uint8_t *body, uint16_t body_len)
{
  const sixp_pkt_type_t type = SIXP_PKT_TYPE_REQUEST;
  const sixp_pkt_code_t code = (sixp_pkt_code_t)(uint8_t)SIXP_PKT_CMD_RELOCATE;
  sixp_pkt_cell_options_t cell_options;
  sixp_pkt_num_cells_t num_cells;
  const uint8_t *relocation_cell_list;
  const uint8_t *candidate_cell_list;
  uint16_t relocation_cell_list_len;
  uint16_t candidate_cell_list_len;

  assert(peer_addr != NULL);

  LOG_INFO("received a RELOCATE request from ");
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(
    sixp_pkt_get_cell_options(type, code, &cell_options, body, body_len) < 0 ||
    sixp_pkt_get_num_cells(type, code, &num_cells, body, body_len) < 0 ||
    sixp_pkt_get_rel_cell_list(type, code,
                               &relocation_cell_list, &relocation_cell_list_len,
                               body, body_len) < 0 ||
    sixp_pkt_get_cand_cell_list(type, code,
                                &candidate_cell_list, &candidate_cell_list_len,
                                body, body_len) < 0) {
    LOG_ERR("parse error\n");
  } else {
    send_response(peer_addr, cell_options, num_cells,
                  relocation_cell_list, relocation_cell_list_len,
                  candidate_cell_list, candidate_cell_list_len);
  }
}
/*---------------------------------------------------------------------------*/
void
msf_sixp_relocate_recv_response(const linkaddr_t *peer_addr, sixp_pkt_rc_t rc,
                                const uint8_t *body, uint16_t body_len)
{
  const uint8_t *cell_list;
  uint16_t cell_list_len;

  assert(peer_addr != NULL);

  LOG_INFO("received a RELOCATE response with %s from ",
           msf_sixp_get_rc_str(rc));
  LOG_INFO_LLADDR(peer_addr);
  LOG_INFO_("\n");

  if(rc == SIXP_PKT_RC_SUCCESS) {
    LOG_INFO("RELOCATE transaction completes successfully\n");
    if(sixp_pkt_get_cell_list(SIXP_PKT_TYPE_RESPONSE,
                              (sixp_pkt_code_t)(uint8_t)SIXP_PKT_RC_SUCCESS,
                              &cell_list, &cell_list_len, body, body_len) < 0) {
      LOG_ERR("parse error\n");
      msf_reserved_cell_delete_all(peer_addr);
    } else if(cell_list_len == 0) {
      LOG_INFO("received an empty CellList; "
               "try another RELOCATE request later\n");
      msf_reserved_cell_delete_all(peer_addr);
    } else if(cell_list_len != sizeof(sixp_pkt_cell_t)) {
      /* invalid length; MSF always requests one cell per RELOCATE request */
      LOG_ERR("received an invalid CellList (%u octets)\n", cell_list_len);
      msf_reserved_cell_delete_all(peer_addr);
    } else {
      uint16_t slot_offset, channel_offset;
      msf_sixp_get_cell_params(cell_list, &slot_offset, &channel_offset);
      if(msf_reserved_cell_get(peer_addr, slot_offset, channel_offset)) {
        /* this is a cell which we proposed in the request */
        msf_reserved_cell_delete_all(peer_addr);
        if(msf_negotiated_cell_add(peer_addr, MSF_NEGOTIATED_CELL_TYPE_TX,
                                   slot_offset, channel_offset) < 0) {
          msf_housekeeping_resolve_inconsistency(peer_addr);
        } else {
          msf_housekeeping_delete_cell_to_relocate();
          /* all good */
        }
      }
    }
  } else {
    LOG_ERR("ADD transaction failed\n");
    msf_reserved_cell_delete_all(peer_addr);
    if(rc == SIXP_PKT_RC_ERR_SEQNUM ||
       rc == SIXP_PKT_RC_ERR_CELLLIST) {
      sixp_trans_abort(sixp_trans_find(peer_addr));
      msf_sixp_clear_send_request(peer_addr);
    } else if(rc == SIXP_PKT_RC_ERR_BUSY) {
      msf_sixp_start_request_wait_timer();
    } else {
      /* do nothing */
    }
  }
}
/*---------------------------------------------------------------------------*/
