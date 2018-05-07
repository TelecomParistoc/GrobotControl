#to lauch the main programm autmaticaly

LAUNCHER = ./robot_autolauncher.sh
AUTOLAUNCH_DIR = /etc/init.d/
#TODO il faut utiliser l'outil de Julien !!!
#ce makefile doit juste etre un lien vers l'outil de julien
install:
	chmod +x $(LAUNCHER)
	cp -a $(LAUNCHER) $(AUTOLAUNCH_DIR)
