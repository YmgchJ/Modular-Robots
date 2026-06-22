#include "agent_id_mapper.h"
#include "pico/unique_id.h"
#include <cstring>

int getAgentId()
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    char uid[17];
    sprintf(uid,
        "%02X%02X%02X%02X%02X%02X%02X%02X",
        id.id[0], id.id[1], id.id[2], id.id[3],
        id.id[4], id.id[5], id.id[6], id.id[7]);
    if (strcmp(uid, "E663A837CB740F2C") == 0) return 1;
    if (strcmp(uid, "E663A837CB8D4A2C") == 0) return 2;
    if (strcmp(uid, "E663A837CB86722A") == 0) return 3;
    if (strcmp(uid, "E663A837CB77502A") == 0) return 4;
    if (strcmp(uid, "E663A837CB88352A") == 0) return 5;
    if (strcmp(uid, "E663A837CB55532A") == 0) return 6;

    return -1;
}