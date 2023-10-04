# GMT Rust Integrated Modeling 

GRSIM is a collection of integrated models based on [`gmt_dos-actors`].

Users should start by reading the [GMT Dos Actors] book and familiarize themselves with [`actorscript`], the `gmt_dos-actors` scripting [language](https://docs.rs/gmt_dos-actors_dsl).

To run any of the model, [Rust] must be installed on the machine.

 
 ## [open-loop](open-loop/README.md)
 
 A simple on-axis GMT optical model with atmospheric turbulence 

 The model is run with:
```shell
cargo run --release --bin open-loop
```

 ## [ngao](ngao/README.md)
 
 GMT NGAO model with pyramid wavefront sensor and holographic dispersed fringe sensor

  The model is run with:
```shell
cargo run --release --bin ngao
```

[`gmt_dos-actors`]: https://crates.io/crates/gmt_dos-actors
[GMT Dos Actors]: https://rconan.github.io/dos-actors/
[`actorscript`]: https://docs.rs/gmt_dos-actors/7.1.2/gmt_dos_actors/macro.actorscript.html
[Rust]: https://www.rust-lang.org/
