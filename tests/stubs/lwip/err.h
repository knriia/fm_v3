#ifndef FM_V3_TEST_LWIP_ERR_H
#define FM_V3_TEST_LWIP_ERR_H

typedef int err_t;

#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_BUF (-2)
#define ERR_CONN (-3)
#define ERR_TIMEOUT (-4)
#define ERR_VAL (-6)
#define ERR_WOULDBLOCK (-7)
#define ERR_ABRT (-13)
#define ERR_RST (-14)
#define ERR_CLSD (-15)
#define ERR_ARG (-16)

#endif /* FM_V3_TEST_LWIP_ERR_H */
