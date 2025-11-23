mv -f ./pixi.toml ./pixi.toml.bak
# crosscompile doesn't work
export arch="architecture"
for distro in jazzy humble kilted; do

    rm ./pixi.toml
    ln -s ./pixi-$distro.toml ./pixi.toml

    pixi build \
        --output-dir ./$arch \
        # --target-platform $arch
done
mv -f ./pixi.toml.bak ./pixi.toml
