#include <board.h>
#include <gs/a3200/a3200.h>
#include <gs/a3200/pwm.h>
#include <gs/a3200/hmc5843.h>
#include <gs/a3200/mpu3300.h>
#include <gs/a3200/spi_slave.h>
#include <gs/a3200/pwr_switch.h>
#include <gs/a3200/adc_channels.h>
#include <gs/a3200/lm71.h>
#include <gs/util/gosh/command.h>
#include <gs/util/gosh/console.h>
#include <gs/util/time.h>
#include <gs/thirdparty/fram/fm33256b.h>
#include <gs/thirdparty/flash/spn_fl512s.h>
#include <gs/embed/command.h>
#include <conf_a3200.h>  // a3200 options - set via wscript
#include "checkout/checkout_cmd.h"

//template for processing incoming csp commands
void csp_task(void *parameters) {
    /* Create socket without any socket options */
    csp_socket_t *sock = csp_socket(CSP_SO_NONE);
    /* Bind all ports to socket */
    csp_bind(sock, CSP_ANY);
    /* Create 10 connections backlog queue */
    csp_listen(sock, 10);
    /* Pointer to current connection and packet */
    csp_conn_t *conn;
    csp_packet_t *packet;
    /* Process incoming connections */
    while (1) {
        /* Wait for connection, 10000 ms timeout */
        if ((conn = csp_accept(sock, 10000)) == NULL)
        continue;
        /* Read packets. Timout is 1000 ms */
        while ((packet = csp_read(conn, 1000)) != NULL) {
            switch (csp_conn_dport(conn)) {
            case MY_PORT: // determine correct port 
            /* Process packet here */
            default:
            /* Let the service handler reply pings, buffer use, etc. */
            csp_service_handler(conn, packet);
            break;
        }
        }
        /* Close current connection, and handle next */
        csp_close(conn);
    }
}

// example csp client function
int send_packet(void) {
    /* Get packet buffer for data */
    csp_packet_t *packet = csp_buffer_get(data_size);
    if (packet == NULL) {
    /* Could not get buffer element */
    printf("Failed to get buffer element\\n");
    return -1;
    }
    /* Connect to host HOST, port PORT with regular UDP-like protocol and 1000 ms timeout*/
    csp_conn_t *conn = csp_connect(CSP_PRIO_NORM, HOST, PORT, 1000, CSP_O_NONE);
    if (conn == NULL) {
        /* Connect failed */
        printf("Connection failed\\n");
        /* Remember to free packet buffer */
        csp_buffer_free(packet);
        return -1;
    }
    /* Copy message to packet */
    char *msg = "HELLO";
    strcpy(packet->data, msg);
    /* Set packet length */
    packet->length = strlen(msg);
    /* Send packet */
    if (!csp_send(conn, packet, 1000)) {
        /* Send failed */
        printf("Send failed\\n");
        csp_buffer_free(packet);
    }
    /* Close connection */
    csp_close(conn);
    return 0
}