## build

```
mkdir build && cd build
cmake .. && make -j4
```

## Use

If the length of the data you receive is fixed and relatively long, you can set vmin

```
    Serial::SerialOpt_t serialOpt {
        .speed = 115200,
        .dataBits = 8,
        .stopbits = 1,
        .parity = 'n',
        .flowControlMode = 's',
        .vtime = 1, // 0.1s * vtime receive timeout
        .vmin = 1,
    };

```