# BSO Gyromouse

Cilj projekta je izdelava brezzicne miske na principu krmilne palice, ki uporablja ziroskop za svoje delovanje.

Uporabljen je senzor MPU-9250 in modul nRF24 za komunikacijo.


## Open FreeRTOS platform

Za uspesno gradnjo projekta je potrebno najprej postaviti razvojno okolje. Skripte so prilagojene da delujejo na modernih sistemih Linux. Na zacetku pozenemo naslednji ukaz: `sh setup-platform.sh`

Za aktivacijo okolja izvedemo ukaz `source ./platform/activate`

Projekte gradimo z ukazi `make clean`, `make -j8` in ga nalozimo z ukazom `make flash`


## Projekt Gyromouse

Osnovni projekt se nahaja v mapi gyromouse. Ta projekt je poskus integracije vseh komponent in lastnih gonilnikov, vendar v casu ki nam je bil na voljo nisva uspela doseci zanesljivega delovanja, zato sva ga razbila na dva osnovna dela in ju izpolnila do delovanja.


## Projekt Gyromouse Lite

Malenkost okrnjena razlicica prvotnega projekta, namen je samo posiljanje koordinat miske in stanja tipk na dongle. S tem sva dobila misko, ki deluje zanesljivo.


## Projekt Gyromouse Dongle

To je razlicica projekta gyromouse, ki sprejema podatke iz modula nRF24 in jih posreduje na serijski vmesnik.


## Python scripts

V projektih se v mapah `scripts` nahajajo Python skripte, ki omogocajo komunikacijo z napravami. Glavna skripta, ki jo potrebujemo je v `gyromouse-dongle/scripts/dongle_driver.py`. Ta omogoca sprejemanje podatkov serijskega vmesnika in interpretacijo v premike miske.

Zahtevani paketi Python:
pyserial, mouse, PyOpenGL, tkinter, pyautogui
