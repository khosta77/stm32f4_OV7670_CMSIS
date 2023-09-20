# stm32f4 работа с камерой OV7670

```
$ make all; sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \
"init; reset halt; flash write_image erase main.hex; "\  
"reset; exit"
```

Добавил две версии библиотек, надо в makefile их переподключать
