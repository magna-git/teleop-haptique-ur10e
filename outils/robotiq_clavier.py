import socket
import time

ROBOT_IP = "169.254.140.20"
PORT = 63352

def commande(sock, texte):
    sock.sendall((texte + "\n").encode("ascii"))
    reponse = sock.recv(1024).decode("ascii").strip()
    print(f"{texte} -> {reponse}")
    return reponse

def regler(sock, texte):
    reponse = commande(sock, texte)
    if reponse.lower() != "ack":
        raise RuntimeError(f"Commande non confirmée : {texte}")
    
with socket.create_connection((ROBOT_IP, PORT), timeout=3) as sock:
    sock.settimeout(3)

    print("=== Commande clavier Robotiq ===")

    etat = commande(sock, "GET STA")

    if etat != "STA 3":
        print("\nLa pince n'est pas prête selon l'interface réseau.")
        choix = input("Autoriser son activation ? (oui/non) : ").strip().lower()

        if choix != "oui":
            print("Test annulé.")
            raise SystemExit

        regler(sock, "SET ACT 1")

        for _ in range(20):
            time.sleep(0.5)
            etat = commande(sock, "GET STA")
            if etat == "STA 3":
                break

        if etat != "STA 3":
            print("Activation non confirmée. Aucun mouvement commandé.")
            raise SystemExit

    print("\nPince prête.")
    print("O = ouvrir | F = fermer | Q = quitter")

    while True:
        touche = input("\nCommande : ").strip().lower()

        if touche == "o":
            regler(sock, "SET SPE 30")
            regler(sock, "SET FOR 30")
            regler(sock, "SET POS 0")
            regler(sock, "SET GTO 1")

        elif touche == "f":
            regler(sock, "SET SPE 30")
            regler(sock, "SET FOR 30")
            regler(sock, "SET POS 255")
            regler(sock, "SET GTO 1")

        elif touche == "q":
            print("Fin du test.")
            break

        else:
            print("Commande inconnue.")
