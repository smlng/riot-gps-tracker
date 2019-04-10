APPLICATION = gps_tracker
RIOTBASE ?= ../RIOT

BOARD=lobaro-lorabox

USEMODULE += isrpipe
USEMODULE += isrpipe_read_timeout
USEMODULE += shell
USEMODULE += xtimer
USEPKG += minmea

# Use by default sx1276 and region EU868
LORA_DRIVER ?= sx1272
LORA_REGION ?= EU868

DEVELHELP=1

# Semtech LoRaMAC pkg is required in order to user LoRaWAN
USEPKG += semtech-loramac

# Load the LoRa Module
USEMODULE += $(LORA_DRIVER)

# We need FMT for handling hex in keys
USEMODULE += fmt

# Tell LoRaMAC pkg which region should be used
CFLAGS += -DREGION_$(LORA_REGION)
CFLAGS += -DLORAMAC_ACTIVE_REGION=LORAMAC_REGION_$(LORA_REGION)

include $(RIOTBASE)/Makefile.include
