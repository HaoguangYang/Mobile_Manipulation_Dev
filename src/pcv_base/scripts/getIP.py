import pymysql.cursors    #For SQL Database

def getIP(NodeNo):
    db = pymysql.connect(
                host = "ldvinmac01.itap.purdue.edu",
                port = 3317,
                user = "Om-students",
                password = "hN$2aIb6L",
                database = "Voyles_robots"
                )
    
    dbName = "Voyles_robots"
    
    TelTableName = "CARTMAN"+NodeNo.zfill(2)+"_TELEMETRY"
    
    
    try:
        with db:
            with db.cursor() as cursor:
                sql = "SELECT `IP` FROM `"+dbName+"`.`"+TelTableName+"` ORDER BY `TimeStamp` DESC LIMIT 1"
                cursor.execute(sql)
                IP = cursor.fetchall()[0][0]
    
    finally:
        if db.open == True:
            db.close()
    
    return IP
    
print(getIP('5'))