// version file, will be touched each build for date and time to be updated.
#include "fraise.h"

void print_version() {
    fraise_printf("V picoethmot @ %s", __DATE__ " " __TIME__ "\n");
}

