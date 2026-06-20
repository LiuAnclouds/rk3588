# Runtime Libraries

Place the RKNN runtime library here on the Orange Pi board:

```text
lib/librknnrt.so
```

The project intentionally uses a project-local RKNN runtime because the system
`/usr/lib/librknnrt.so` may be too old for newer RKNN model versions.

Binary `.so` files are ignored by Git.

