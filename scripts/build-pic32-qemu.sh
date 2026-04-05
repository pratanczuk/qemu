#!/usr/bin/env bash
# Rebuild qemu-system-mipsel with current sources. Writes to build-pic32-qemu/
# (in-tree mipsel-softmmu/ is often root-owned from an old /qemu configure.)
set -euo pipefail
QROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$QROOT/build-pic32-qemu"
TMP=/tmp/mipsel-sm-build-$$
mkdir -p "$OUT"
cp "$QROOT/config-host.mak" /tmp/config-host.mak.$$
sed -i "s|^SRC_PATH=.*|SRC_PATH=$QROOT|" /tmp/config-host.mak.$$
sed -i "s|-I/qemu/linux-headers|-I$QROOT/linux-headers|g" /tmp/config-host.mak.$$
sed -i 's|PYTHON=/usr/bin/python2|PYTHON=/usr/bin/python3|' /tmp/config-host.mak.$$
mv /tmp/config-host.mak.$$ /tmp/config-host.mak

rm -rf "$TMP"
cp -a "$QROOT/mipsel-softmmu" "$TMP"
rm -f "$TMP/Makefile"
ln -sf "$QROOT/Makefile.target" "$TMP/Makefile"

# make -p exits 2 even when the database prints; with pipefail that would abort set -e.
set +o pipefail
DEPS=$(cd "$TMP" && make SRC_PATH="$QROOT" -p 2>/dev/null | grep '^qemu-system-mipsel:' | sed 's/^qemu-system-mipsel: //')
set -o pipefail
for p in $DEPS; do
  case "$p" in
    ../*)
      rel="${p#../}"
      mkdir -p "/tmp/$(dirname "$rel")"
      if [[ ! -e "/tmp/$rel" && -e "$QROOT/$rel" ]]; then
        ln -sf "$QROOT/$rel" "/tmp/$rel"
      fi
      ;;
  esac
done

cp -a "$QROOT/mipsel-softmmu/trace/"* "$TMP/trace/" 2>/dev/null || true
touch "$TMP/trace/generated-helpers.c-timestamp" "$TMP/trace/generated-helpers.h-timestamp" "$TMP/trace/generated-tcg-tracers.h-timestamp"
touch -r "$QROOT/trace-events" "$TMP/trace/generated-helpers.c-timestamp"
touch -d '+1 day' "$TMP/trace/generated-helpers.c-timestamp" "$TMP/trace/generated-helpers.h-timestamp" "$TMP/trace/generated-tcg-tracers.h-timestamp"

find "$TMP" -name '*.d' -delete
rm -f "$TMP/hw/mips/mips_pic32mx3.d"
(
  cd "$TMP"
  make SRC_PATH="$QROOT" hw/mips/mips_pic32mx3.o qemu-system-mipsel -j"$(nproc)"
)
cp -f "$TMP/qemu-system-mipsel" "$OUT/qemu-system-mipsel"
chmod +x "$OUT/qemu-system-mipsel"
rm -rf "$TMP"
echo "OK: $OUT/qemu-system-mipsel"
