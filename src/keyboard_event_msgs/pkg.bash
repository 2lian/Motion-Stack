for arch in linux-64 linux-aarch64; do
    for distro in jazzy humble kilted; do
        rm -r ./pixi-$distro/package.xml
        rm -r ./pixi-$distro/CMakeLists.txt
        rm -r ./pixi-$distro/msg

        ln -s ../package.xml      ./pixi-$distro/package.xml
        ln -s ../CMakeLists.txt   ./pixi-$distro/CMakeLists.txt
        ln -s ../msg              ./pixi-$distro/msg

        pixi build \
            --manifest-path ./pixi-$distro/pixi.toml \
            --output-dir ./$arch \
            --target-platform $arch
    done
done
