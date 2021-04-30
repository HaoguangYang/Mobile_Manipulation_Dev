import pymysql.cursors    #For SQL Database

def LastTimeStamp(NodeNo):
    
    #Connection to the Database
    db = pymysql.connect(
                host = "ldvinmac01.itap.purdue.edu",
                port = 3317,
                user = "Om-students",
                password = "hN$2aIb6L",
                database = "Voyles_robots"
                )
    
    dbName = "Voyles_robots"
    NavTableName = "CARTMAN"+NodeNo.zfill(2)+"_NAV"
    
    try:
        with db:
            with db.cursor() as cursor:
                sql = "SELECT `TimeStamp` FROM `"+dbName+"`.`"+NavTableName+"` ORDER BY `TimeStamp` DESC LIMIT 1"
                cursor.execute(sql)
                LastTimeStamp=cursor.fetchall()
                print(f'Last TimeStamp: {LastTimeStamp[0][0]}')
            
    finally:
        if db.open == True:
            db.close()
            
LastTimeStamp('5')