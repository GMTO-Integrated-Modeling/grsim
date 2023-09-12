# GRISM Open-Loop

 A simple on-axis GMT optical model with atmospheric turbulence 

 The model is run with:
```shell
cargo run --release --bin open-loop
```

The scope client is run with:
```shell
cargo run -p grsim_open-loop --release --features scope-client --bin scope-WfeRms
```