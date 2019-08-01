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

#include <contiki.h>

#include <net/netstack.h>
#include <net/ipv6/uip.h>
#include <net/ipv6/uip-ds6-route.h>
#include <net/ipv6/uip-icmp6.h>
#include <sys/log.h>

#include <services/shell/shell.h>
#include <services/shell/shell-commands.h>
#include <services/shell/serial-shell.h>

#ifndef BUILD_WITH_RPL_BORDER_ROUTER
#error BUILD_WITH_RPL_BORDER_ROUTER required
#endif /* !BUILD_WITH_RPL_BORDER_ROUTER */

#define LOG_MODULE "APP"
#define LOG_LEVEL LOG_LEVEL_MAIN

static struct shell_command_set_t custom_shell_command_set;

extern int contiki_argc;
extern char **contiki_argv;
extern int slip_config_handle_arguments(int argc, char **argv);
extern const struct network_driver tun6_net_driver;

static void tun6_net_driver_init(void);
static int tun6_net_driver_output(void);
static void init_routing_table(void);
static void rs_input(void);

UIP_ICMP6_HANDLER(rs_input_handler, ICMP6_RS,
                  UIP_ICMP6_HANDLER_CODE_ANY, rs_input);
PROCESS(standalone_rpl_border_router_process,
        "Standalone RPL Border Router Process");
AUTOSTART_PROCESSES(&standalone_rpl_border_router_process);

/*---------------------------------------------------------------------------*/
static void
tun6_net_driver_init(void)
{
  tun6_net_driver.init();
}
/*---------------------------------------------------------------------------*/
static int
tun6_net_driver_output(void)
{
  const linkaddr_t *localdest_null = NULL;
  return tun6_net_driver.output(localdest_null);
}
/*---------------------------------------------------------------------------*/
static void
init_routing_table(void)
{
  uip_ds6_defrt_t *defrt = uip_ds6_defrt_head();
  uip_ds6_defrt_t *defrt_next;
  const uip_ip6addr_t *prefix;
  const uint8_t prefix_len = 8;

  /* remove all the default routes if any */
  while(defrt != NULL) {
    defrt_next = defrt->next;
    LOG_INFO("Remove a default router entry of ");
    LOG_INFO_6ADDR(&defrt->ipaddr);
    LOG_INFO_("\n");
    uip_ds6_defrt_rm(defrt);
    defrt = defrt_next;
  }

  /*
   * leave the default route table empty so as to route packets going
   * to backhaul to the TUN interface
   */

  /*
   * add a dummy route so that packets destined to the RPL network go
   * to SLIP
   */
  LOG_INFO("Add a dummy route for packets destined to the RPL network\n");
  prefix = uip_ds6_default_prefix();
  uip_ds6_nbr_add(prefix, (uip_lladdr_t *)&linkaddr_null, 1,
                  NBR_REACHABLE, NBR_TABLE_REASON_IPV6_ND_AUTOFILL, NULL);
  uip_ds6_route_add(prefix, prefix_len, prefix);
}
/*---------------------------------------------------------------------------*/
static void
rs_input(void)
{
  LOG_INFO("Ignore RS from ");
  LOG_INFO_6ADDR(&UIP_IP_BUF->srcipaddr);
  LOG_INFO_("\n");
  uipbuf_clear();
}
/*---------------------------------------------------------------------------*/
/*
 * The prototype of rpl_border_router_init() is defined in
 * os/services/rpl-border-router/rpl-border-router.h, which is
 * included by contiki-main.c.
 */
void
rpl_border_router_init(void)
{
  serial_shell_init();
  shell_command_set_register(&custom_shell_command_set);
  uip_icmp6_register_input_handler(&rs_input_handler);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(standalone_rpl_border_router_process, ev, data)
{
  PROCESS_BEGIN();

  (void)slip_config_handle_arguments(contiki_argc, contiki_argv);

  init_routing_table();
  LOG_INFO("Start as RPL Root\n");
  NETSTACK_ROUTING.root_start();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static
PT_THREAD(cmd_rpl_set_root(struct pt *pt, shell_output_func output, char *args))
{
  PT_BEGIN(pt);
  SHELL_OUTPUT(output,
               "rpl-set-root is not supported "
               "by standalone-rpl-border-router\n");
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
const struct shell_command_t custom_shell_commands[] = {
  { "rpl-set-root", cmd_rpl_set_root, NULL },
  { NULL, NULL, NULL }
};
/*---------------------------------------------------------------------------*/
static struct shell_command_set_t custom_shell_command_set = {
  .next = NULL,
  .commands = custom_shell_commands,
};
/*---------------------------------------------------------------------------*/
const struct uip_fallback_interface tun6_interface = {
  tun6_net_driver_init,
  tun6_net_driver_output
};
/*---------------------------------------------------------------------------*/
