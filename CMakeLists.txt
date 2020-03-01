# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BH1750)

message("UMDOM: modul BH1750 Запуск сборки.")

zephyr_include_directories( . )


zephyr_library()

zephyr_library_sources(bh1750.c)
    
endif(CONFIG_BH1750)

