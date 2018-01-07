set -euxo pipefail

main() {
    if [ $TARGET = x86_64-unknown-linux-gnu ]; then
        cargo check --target $TARGET
        return
    fi
    xargo clean
    xargo check --target $TARGET
    xargo check --target $TARGET --examples
}

main