# NGAO

 GMT NGAO model with pyramid wavefront sensor and holographic dispersed fringe sensor

  The model is run with:
```shell
cargo run --release --bin ngao
```

The scopes client are run with:
```shell
cargo run -p grsim_ngao --release --features scope-client --bin scope-WfeRms
```
and with 
```shell
cargo run -p grsim_ngao --release --features scope-client --bin scope-SegmentPiston
```