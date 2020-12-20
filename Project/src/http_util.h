#define HTTP_PORT 8000
#define HTTP_PROTOCOL "HTTP/1.2"

#define MAX_RECV_BUF_LEN 512

#define REMOTE_SERVER_ADDRESS  CONFIG_NET_CONFIG_PEER_IPV4_ADDR

int ping_http_server();
int post_sensor_data(char *data);
