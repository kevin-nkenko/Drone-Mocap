#-*- coding: utf-8 -*-

#generation du fichier nodeJs pour le drone

def genererFichierVol():
    suitecommandes="var arDrone = require('ar-drone');\nvar client  = arDrone.createClient();\n \n client.takeoff(); \n "
    suitecommandes+=planning

    fichierVol = open("fichierVol.js", "w")
    fichierVol.write(suitecommandes)
    fichierVol.close()