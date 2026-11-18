idf_component_register(
    SRCS "tx_backlog.c"
         "tx_backlog_sock.c"
    INCLUDE_DIRS "include"
    # The public header exposes plain C types only. lwIP is for the socket
    # push (tx_backlog_sock.c); the ring and the pool (tx_backlog.c) need
    # nothing and are what the host tests compile.
    PRIV_REQUIRES lwip
)
