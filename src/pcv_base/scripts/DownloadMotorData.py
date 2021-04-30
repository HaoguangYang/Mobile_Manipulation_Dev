import pymysql.cursors    #For SQL Database

def DownloadMotorData(NodeNo):
    
    #Connection to the Database
    db = pymysql.connect(
                host = "ldvinmac01.itap.purdue.edu",
                port = 3317,
                user = "Om-students",
                password = "hN$2aIb6L",
                database = "Voyles_robots"
                )
    
    dbName = "Voyles_robots"
    
    MotorTableName = "CARTMAN"+NodeNo.zfill(2)+"_MOTOR"
    NavTableName = "CARTMAN"+NodeNo.zfill(2)+"_NAV"
    
    
    try:
        with db:
            with db.cursor() as cursor:
                sql_Motor = "SELECT * FROM `"+dbName+"`.`"+MotorTableName+"` ORDER BY `TimeStamp` DESC LIMIT 1"
                cursor.execute(sql_Motor)
                MotorData = cursor.fetchall()
                TimeStampNav = MotorData[0][0]
                
                steer1Amp = MotorData[0][1]
                steer1AMax = MotorData[0][2]
                roll1Amp = MotorData[0][3]
                roll1AMax = MotorData[0][4]
                
                steer2Amp = MotorData[0][5]
                steer2AMax = MotorData[0][6]
                roll2Amp = MotorData[0][7]
                roll2AMax = MotorData[0][8]
                
                steer3Amp = MotorData[0][9]
                steer3AMax = MotorData[0][10]
                roll3Amp = MotorData[0][11]
                roll3AMax = MotorData[0][12]
                
                steer4Amp = MotorData[0][13]
                steer4AMax = MotorData[0][14]
                roll4Amp = MotorData[0][15]
                roll4AMax = MotorData[0][16]
                
                
                sql_Nav = "SELECT * FROM `"+dbName+"`.`"+NavTableName+"` ORDER BY `TimeStamp` DESC LIMIT 1"
                cursor.execute(sql_Nav)
                NavData = cursor.fetchall()
                TimeStampNav = NavData[0][0]
                
                PosX = NavData[0][1]
                PosY = NavData[0][2]
                Orientation = NavData[0][3]
                VelX = NavData[0][4]
                VelY = NavData[0][5]
                AngVel = NavData[0][6]
                DesX = NavData[0][7]
                DesY = NavData[0][8]
                DesOrient = NavData[0][9]
                NavStatus = NavData[0][10]
                
  
                
    finally:
        if db.open == True:
            db.close()
            
DownloadMotorData('5')
