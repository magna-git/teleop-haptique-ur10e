import ctypes

LIB = "/home/user/sdk-3.17.7/lib/release/lin-x86_64-gcc/libdrd.so.3.17.7"

sdk = ctypes.CDLL(LIB)

sdk.drdOpen.argtypes = []
sdk.drdOpen.restype = ctypes.c_int

sdk.drdClose.argtypes = [ctypes.c_int]
sdk.drdClose.restype = ctypes.c_int

sdk.dhdGetPosition.argtypes = [
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.POINTER(ctypes.c_double),
    ctypes.c_int
]
sdk.dhdGetPosition.restype = ctypes.c_int


def position(dev):
    x = ctypes.c_double()
    y = ctypes.c_double()
    z = ctypes.c_double()

    result = sdk.dhdGetPosition(
        ctypes.byref(x),
        ctypes.byref(y),
        ctypes.byref(z),
        dev
    )

    if result < 0:
        raise RuntimeError("Erreur de lecture omega.6")

    return [x.value, y.value, z.value]


dev = sdk.drdOpen()

if dev < 0:
    raise RuntimeError("omega.6 non détecté")

tests = [
    "VERS LE HAUT",
    "VERS TA DROITE",
    "VERS L'AVANT (loin de toi)"
]

try:
    print("\n=== IDENTIFICATION DES AXES OMEGA.6 ===")

    for test in tests:
        print(f"\n--- TEST : {test} ---")

        input("Ramène la poignée au centre, puis appuie sur Entrée : ")
        p0 = position(dev)

        input(
            f"Déplace la poignée {test}, "
            "maintiens-la, puis appuie sur Entrée : "
        )
        p1 = position(dev)

        delta = [
            1000 * (p1[i] - p0[i])
            for i in range(3)
        ]

        labels = ["X", "Y", "Z"]
        dominant = max(range(3), key=lambda i: abs(delta[i]))

        print("\nDéplacement mesuré :")

        for label, value in zip(labels, delta):
            print(f"{label} = {value:+.1f} mm")

        print(
            f"AXE DOMINANT : {labels[dominant]} "
            f"({'+' if delta[dominant] >= 0 else '-'})"
        )

    print("\nTests terminés !")

finally:
    sdk.drdClose(dev)
    print("omega.6 déconnecté.")
