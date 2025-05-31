
/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

CREATE DATABASE /*!32312 IF NOT EXISTS*/ `bconnectpoc` /*!40100 DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci */ /*!80016 DEFAULT ENCRYPTION='N' */;

USE `bconnectpoc`;
DROP TABLE IF EXISTS `bcoresmartservice`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `bcoresmartservice` (
  `serviceID` int NOT NULL DEFAULT '1',
  `comments` varchar(100) NOT NULL DEFAULT 'On Start Data',
  `serviceDate` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`serviceID`),
  UNIQUE KEY `serviceID_UNIQUE` (`serviceID`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `commoncode`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `commoncode` (
  `COMP_CD` varchar(10) NOT NULL,
  `MAJOR_CD` varchar(10) NOT NULL,
  `MINOR_CD` varchar(10) NOT NULL,
  `CD_NM` varchar(100) DEFAULT NULL,
  `DEFAULT_FLAG` varchar(1) DEFAULT '0',
  `SORT_NO` varchar(45) DEFAULT '0',
  `REMARK` varchar(200) DEFAULT NULL,
  `REL_CD1` varchar(60) DEFAULT NULL,
  `REL_CD2` varchar(60) DEFAULT NULL,
  `REL_CD3` varchar(60) DEFAULT NULL,
  `REL_CD4` varchar(60) DEFAULT NULL,
  `REL_CD5` varchar(60) DEFAULT NULL,
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`COMP_CD`,`MAJOR_CD`,`MINOR_CD`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `currtasksetting`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `currtasksetting` (
  `CurrentTaskID` int NOT NULL AUTO_INCREMENT,
  `taskid` int NOT NULL DEFAULT '0',
  `robotURL` varchar(100) DEFAULT NULL,
  `simulayoutcds` varchar(45) DEFAULT NULL,
  `startnodes` varchar(45) DEFAULT NULL,
  `endnodes` varchar(45) DEFAULT NULL,
  `currentnodes` varchar(45) DEFAULT NULL,
  `curtaskid` varchar(45) DEFAULT NULL,
  `curtaskdtlid` varchar(45) DEFAULT NULL,
  `tasktype` varchar(45) DEFAULT NULL COMMENT '0-dummytask.1-Actual Task',
  `start_station` varchar(45) DEFAULT NULL,
  `final_station` varchar(45) DEFAULT '0',
  `orderstatus` varchar(45) DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `workstatus` varchar(45) DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `extraservicerun` varchar(10) DEFAULT '1' COMMENT 'Extraservice Run  example like Collect empty trash,Cash Pay and other extra service Run or not , 0  for not run and 1 for Run',
  `column1` varchar(45) DEFAULT NULL COMMENT 'Rack',
  `column2` varchar(45) DEFAULT NULL,
  `column3` varchar(45) DEFAULT NULL,
  `column4` varchar(45) DEFAULT NULL,
  `column5` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`CurrentTaskID`),
  UNIQUE KEY `CurrentTaskID_UNIQUE` (`CurrentTaskID`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `fileuploads`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `fileuploads` (
  `fid` int NOT NULL AUTO_INCREMENT,
  `filename` varchar(200) DEFAULT NULL,
  `filepath` varchar(500) DEFAULT NULL,
  `cols1` varchar(45) DEFAULT '0',
  PRIMARY KEY (`fid`),
  UNIQUE KEY `fid_UNIQUE` (`fid`)
) ENGINE=InnoDB AUTO_INCREMENT=25 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `mapjson`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `mapjson` (
  `idmapjson` int NOT NULL AUTO_INCREMENT,
  `templatecode` varchar(45) DEFAULT NULL,
  `mapjson` longtext,
  PRIMARY KEY (`idmapjson`),
  UNIQUE KEY `idmapjson_UNIQUE` (`idmapjson`)
) ENGINE=InnoDB AUTO_INCREMENT=9 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `menumaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `menumaster` (
  `MenuIdentity` int NOT NULL AUTO_INCREMENT,
  `MenuID` varchar(45) NOT NULL,
  `MenuName` varchar(45) NOT NULL,
  `Parent_MenuID` varchar(45) NOT NULL,
  `RoleID` int NOT NULL,
  `MenuKRNM` varchar(45) NOT NULL,
  `MenuENNM` varchar(100) NOT NULL,
  `MenuURL` varchar(100) NOT NULL,
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `menuClass` varchar(100) DEFAULT NULL,
  PRIMARY KEY (`MenuIdentity`,`MenuID`,`MenuName`,`Parent_MenuID`,`RoleID`),
  UNIQUE KEY `MenuIdentity_UNIQUE` (`MenuIdentity`)
) ENGINE=InnoDB AUTO_INCREMENT=167 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotalarmmaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotalarmmaster` (
  `ipaddress` varchar(20) NOT NULL,
  `alarmcode` varchar(20) NOT NULL,
  `alarmmsg` varchar(200) NOT NULL,
  `comments` varchar(45) DEFAULT NULL,
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotmaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotmaster` (
  `robotmID` int NOT NULL AUTO_INCREMENT,
  `robotID` int NOT NULL,
  `robotmodelno` varchar(45) NOT NULL,
  `robotmipaddress` varchar(45) NOT NULL,
  `robotmhostnm` varchar(45) NOT NULL,
  `robotsipaddress` varchar(45) NOT NULL,
  `robotshostnm` varchar(45) NOT NULL,
  `minChargeValue` int NOT NULL DEFAULT '30',
  `maxChargeValue` int NOT NULL,
  `chkchargectatus` varchar(45) NOT NULL DEFAULT 'Y',
  `Comments` varchar(100) DEFAULT '',
  `burl1` varchar(100) NOT NULL DEFAULT '',
  `burl2` varchar(100) NOT NULL DEFAULT '',
  `burl3` varchar(100) NOT NULL DEFAULT '',
  `burl4` varchar(100) NOT NULL DEFAULT '',
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT '0',
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT '0',
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`robotmID`,`robotmodelno`,`robotmipaddress`),
  UNIQUE KEY `robotmodelno_UNIQUE` (`robotmodelno`),
  UNIQUE KEY `robotmipaddress_UNIQUE` (`robotmipaddress`)
) ENGINE=InnoDB AUTO_INCREMENT=57 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotnames`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotnames` (
  `robotid` int unsigned NOT NULL AUTO_INCREMENT,
  `robotname` varchar(100) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci DEFAULT NULL,
  `robottype` varchar(20) DEFAULT '',
  `Comments` varchar(200) DEFAULT '',
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`robotid`),
  UNIQUE KEY `robotname_UNIQUE` (`robotname`)
) ENGINE=InnoDB AUTO_INCREMENT=39 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotorderexecdetail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotorderexecdetail` (
  `orderexecdid` varchar(45) NOT NULL,
  `orderexecmid` varchar(45) NOT NULL,
  `startworkstation` varchar(10) DEFAULT NULL,
  `finalworkstation` varchar(10) DEFAULT NULL,
  `completestatus` varchar(45) DEFAULT '1' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `workstatus` varchar(10) NOT NULL DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `comments` varchar(200) DEFAULT NULL,
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`orderexecdid`,`orderexecmid`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotorderexecmaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotorderexecmaster` (
  `orderexecmid` varchar(40) NOT NULL,
  `taskid` int NOT NULL DEFAULT '0',
  `robotip` varchar(40) NOT NULL,
  `workname` varchar(100) NOT NULL,
  `tasktype` varchar(10) DEFAULT '1' COMMENT '0-HomeEmergencyCall,1- Parking, 2- Serving Task,3-Cash Pay ,4 - Collecting Empty plattes,5 - returnHomeaftereachtask',
  `ordertype` varchar(10) DEFAULT '1' COMMENT '1-MannualWorkStarted,2-AutoWorkStarted',
  `orderstatus` varchar(10) DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `comments` varchar(200) DEFAULT NULL,
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`orderexecmid`,`robotip`,`workname`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotsimgraph`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotsimgraph` (
  `graphid` int NOT NULL AUTO_INCREMENT,
  `simulayoutcd` varchar(15) NOT NULL,
  `startnode` varchar(45) NOT NULL,
  `endnode` varchar(45) NOT NULL,
  `distance` decimal(10,2) DEFAULT '0.00',
  `reachtime` decimal(10,2) DEFAULT '0.00',
  `speed` decimal(10,2) DEFAULT '0.00',
  `iconxval` decimal(10,2) DEFAULT '0.00',
  `iconyval` decimal(10,2) DEFAULT '0.00',
  `linexval` decimal(10,2) DEFAULT '0.00',
  `lineyval` decimal(10,2) DEFAULT '0.00',
  `directions` varchar(45) DEFAULT 'S',
  `rail_type` varchar(45) DEFAULT 'N' COMMENT 'N- Normal Line , LS-logirthemic arc ,ES - Exponential arc',
  `nodetype` varchar(45) DEFAULT 'HR' COMMENT 'H-Home,R-Rotation,HR-HomeandRotation.NRNodeandRotation,PR-ParkingandRotation',
  `comments` varchar(100) DEFAULT '',
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT '0',
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT '0',
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`graphid`),
  UNIQUE KEY `graphid_UNIQUE` (`graphid`)
) ENGINE=InnoDB AUTO_INCREMENT=3209 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotsimorderexecdetail`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotsimorderexecdetail` (
  `orderexecdid` bigint NOT NULL AUTO_INCREMENT,
  `orderexecmid` bigint NOT NULL,
  `startworkstation` varchar(10) DEFAULT NULL,
  `finalworkstation` varchar(45) DEFAULT NULL,
  `completestatus` varchar(45) DEFAULT '1' COMMENT '1 -  Order Started , -2- Order Running , 3 -  Order  Completed , 4 -  Order  Interrupted',
  `workstatus` varchar(10) NOT NULL DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 ,Running=2,Suspended=3,Completed=4,Failed=5,Canceled=6,OverTime=7,NotFound=404',
  `comments` varchar(200) DEFAULT NULL,
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`orderexecdid`,`orderexecmid`)
) ENGINE=InnoDB AUTO_INCREMENT=7438 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotsimorderexecmaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotsimorderexecmaster` (
  `orderexecmid` bigint NOT NULL,
  `workname` varchar(100) NOT NULL,
  `robotip` varchar(40) NOT NULL,
  `ordertype` varchar(10) DEFAULT '1' COMMENT '1-MannualWorkStarted,2-AutoWorkStarted',
  `orderstatus` varchar(10) DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 ,Running=2,Suspended=3,Completed=4,Failed=5,Canceled=6,OverTime=7,NotFound=404',
  `comments` varchar(200) DEFAULT NULL,
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`orderexecmid`,`workname`,`robotip`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `robotsimtablegraph`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `robotsimtablegraph` (
  `graphid` int NOT NULL AUTO_INCREMENT,
  `simulayoutcd` varchar(15) NOT NULL,
  `tablearuco` varchar(45) NOT NULL,
  `custtableno` varchar(100) DEFAULT NULL,
  `endnode` varchar(45) NOT NULL,
  `distance` decimal(10,2) DEFAULT '0.00',
  `reachtime` decimal(10,2) DEFAULT '0.00',
  `speed` decimal(10,2) DEFAULT '0.00',
  `tasktype` varchar(45) DEFAULT NULL COMMENT 'A007',
  `taskdetailtype` varchar(45) DEFAULT NULL COMMENT 'A008',
  `iconxval` decimal(10,2) DEFAULT '0.00',
  `iconyval` decimal(10,2) DEFAULT '0.00',
  `linexval` decimal(10,2) DEFAULT '0.00',
  `lineyval` decimal(10,2) DEFAULT '0.00',
  `directions` varchar(45) DEFAULT 'S',
  `tableangle` varchar(45) DEFAULT NULL,
  `comments` varchar(100) DEFAULT '',
  `priyorityuse` varchar(45) NOT NULL DEFAULT '1' COMMENT '0 - Strictly use this path , 1 - Strictly use this path',
  `IsApporved` varchar(45) NOT NULL DEFAULT '1' COMMENT '0-Inactive , 1- Active',
  `CreatedBY` int DEFAULT '0',
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT '0',
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `column1` varchar(45) DEFAULT NULL,
  `column2` varchar(45) DEFAULT NULL,
  `column3` varchar(45) DEFAULT NULL,
  `column4` varchar(45) DEFAULT NULL,
  `column5` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`graphid`),
  UNIQUE KEY `graphid_UNIQUE` (`graphid`)
) ENGINE=InnoDB AUTO_INCREMENT=2145 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `rolemaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `rolemaster` (
  `RoleID` int NOT NULL AUTO_INCREMENT,
  `RoleName` varchar(50) NOT NULL,
  `Comments` varchar(50) DEFAULT NULL,
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  PRIMARY KEY (`RoleID`,`RoleName`),
  UNIQUE KEY `RoleID_UNIQUE` (`RoleID`),
  UNIQUE KEY `RoleName_UNIQUE` (`RoleName`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `taskchain`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `taskchain` (
  `taskid` int NOT NULL AUTO_INCREMENT,
  `robotip` varchar(40) NOT NULL,
  `workname` varchar(10) NOT NULL,
  `simulayoutcds` varchar(45) DEFAULT NULL,
  `startworkstation` varchar(10) DEFAULT NULL,
  `endworkstation` varchar(10) DEFAULT NULL,
  `trayrack` varchar(10) DEFAULT NULL,
  `taskrunok` varchar(10) DEFAULT NULL COMMENT '0- cash and collect not fucntion , 1- All task Function',
  `tasktype` varchar(10) DEFAULT '1' COMMENT '0-HomeCall,1- Parking, 2- Serving Task,3-Cash ,4 - Collecting Empty plattes ,5 - returnHome',
  `orderstatus` varchar(10) DEFAULT '0' COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Canceled=7, NotFound=404',
  `priority` varchar(10) DEFAULT '0' COMMENT '0 - Low ,1- Normal ,2- High , 3-Highest',
  `emergency` varchar(10) DEFAULT '0' COMMENT ' 0-no ,1 -yes',
  `sortorder` varchar(10) DEFAULT NULL,
  `comments` varchar(200) DEFAULT NULL COMMENT 'StatusNone=0, Waiting=1 Started=2,Running=3,Paused=4 ,Failed=5, Completed=6, Cancelled=7, Order Return Not Pickup = 8 , NotFound=404',
  `CreatedBY` int DEFAULT NULL,
  `CreatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `UpdatedBy` int DEFAULT NULL,
  `UpdatedDate` timestamp NULL DEFAULT CURRENT_TIMESTAMP,
  `column1` varchar(45) DEFAULT NULL,
  `column2` varchar(45) DEFAULT NULL,
  `column3` varchar(45) DEFAULT NULL,
  `column4` varchar(45) DEFAULT NULL,
  `column5` varchar(45) DEFAULT NULL,
  PRIMARY KEY (`taskid`)
) ENGINE=InnoDB AUTO_INCREMENT=206 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `texttospeechplay`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `texttospeechplay` (
  `texttoplay` varchar(150) NOT NULL,
  `langtype` varchar(45) NOT NULL DEFAULT 'ko-KR' COMMENT '''ko-KR'' for Korean , ''en-US'' for English ',
  `noofplays` varchar(45) NOT NULL DEFAULT '1',
  `playstatus` varchar(45) DEFAULT '0' COMMENT '0 for not playing and 1 for the text to speech play',
  `col1` varchar(45) DEFAULT NULL COMMENT 'Optional Column1 col1s=1 (0 as default and Send the value as 1 in order to show the video in centre)\n',
  `col2` varchar(45) DEFAULT NULL COMMENT 'Column 2 for the later use'
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci COMMENT='texttospeech';
/*!40101 SET character_set_client = @saved_cs_client */;
DROP TABLE IF EXISTS `usermaster`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `usermaster` (
  `UserId` int unsigned NOT NULL AUTO_INCREMENT,
  `UserName` varchar(100) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci NOT NULL,
  `RoleID` int NOT NULL,
  `Password` varchar(200) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci NOT NULL,
  `EMPNO` varchar(40) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci DEFAULT NULL,
  `UserDispName` varchar(50) NOT NULL,
  `Email` varchar(50) DEFAULT NULL,
  `PHNO` varchar(20) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci DEFAULT NULL,
  `Address` varchar(200) CHARACTER SET utf8mb3 COLLATE utf8mb3_general_ci DEFAULT NULL,
  `IsApporved` int DEFAULT '0' COMMENT '0 - Not approved , 1 - Approved,2-Hold',
  `Status` int DEFAULT '0' COMMENT '0-Inactive , 1- Active',
  PRIMARY KEY (`UserId`,`UserName`,`RoleID`),
  UNIQUE KEY `UserName_UNIQUE` (`UserName`),
  UNIQUE KEY `UserId_UNIQUE` (`UserId`)
) ENGINE=InnoDB AUTO_INCREMENT=17 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci ROW_FORMAT=DYNAMIC;
/*!40101 SET character_set_client = @saved_cs_client */;
/*!50003 DROP PROCEDURE IF EXISTS `sp_bcoreserviceupdate` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_bcoreserviceupdate`(
IN serviceIDs varchar(50),
IN commentss varchar(100), 
IN SPstatus varchar(10)
)
BEGIN 
		IF SPstatus='1' THEN
					SELECT '0' as RowsVal,
								`bcoresmartservice`.`serviceID`,
								`bcoresmartservice`.`comments`,
								`bcoresmartservice`.`serviceDate`
							FROM  `bcoresmartservice`;
        END IF;  
        
        IF SPstatus='2' THEN
        
        IF EXISTS(SELECT 1 FROM   `bcoresmartservice` WHERE serviceID = serviceIDs) THEN
					UPDATE  `bcoresmartservice`
							SET 
							`comments` = commentss ,
                            `serviceDate` = CURRENT_TIMESTAMP
							WHERE `serviceID` = serviceIDs;
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'Roles data updated' as VALUE;

        ELSE
						INSERT INTO  `bcoresmartservice`
									(`serviceID`,
									`comments`,
                                     `serviceDate`)
									VALUES
									(serviceIDs,
									commentss,
                                    CURRENT_TIMESTAMP);
						Select 'I101' as CODE,'okSaved' as MSG ,'Roles data inserted' as VALUE;

        END IF;
        
        END IF;  
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_commoncodeGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_commoncodeGetPost`(  IN COMP_CDs varchar(10),
IN MAJOR_CDs varchar(10),
IN MINOR_CDs varchar(10),
IN CD_NMs varchar(60),
IN DEFAULT_FLAGs varchar(1),
IN SORT_NOs varchar(10), 
IN REMARKs varchar(100),
IN REL_CD1s varchar(60),
IN REL_CD2s varchar(60),
IN REL_CD3s varchar(60),
IN REL_CD4s varchar(60),
IN REL_CD5s varchar(60),
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  
)
BEGIN		 

 	IF SPstatus='1' THEN
                
                SELECT 	 
							case when A.MAJOR_CD IS NULL or A.MAJOR_CD = '' then '' else A.MAJOR_CD  end  MAJOR_CD, 
                            case when A.CD_NM IS NULL or A.CD_NM = '' then '' else A.CD_NM  end  CD_NM  ,
                            case when A.REMARK IS NULL or A.REMARK = '' then '' else A.REMARK  end  REMARK
				FROM  `commoncode`  as A
							 
                    Where 		 A.MAJOR_CD like CONCAT('%', MAJOR_CDs, '%')   
							AND  A.CD_NM like CONCAT('%', CD_NMs, '%')   
							AND  A.MINOR_CD = '*****'   
                        order by  CAST(sort_No AS SIGNED);
                  END IF;
		 
				IF SPstatus='2' THEN
                
                SELECT 		'0' as RowsVal,
							case when A.COMP_CD IS NULL or A.COMP_CD = '' then '' else A.COMP_CD  end  COMP_CD,
							case when A.MAJOR_CD IS NULL or A.MAJOR_CD = '' then '' else A.MAJOR_CD  end  MAJOR_CD,
                            case when A.MINOR_CD IS NULL or A.MINOR_CD = '' then '' else A.MINOR_CD  end  MINOR_CD,
                            case when A.CD_NM IS NULL or A.CD_NM = '' then '' else A.CD_NM  end  CD_NM,
                            case when A.DEFAULT_FLAG IS NULL or A.DEFAULT_FLAG = '' then '' else A.DEFAULT_FLAG  end  DEFAULT_FLAG,
                            case when A.SORT_NO IS NULL or A.SORT_NO = '' then '' else A.SORT_NO  end  SORT_NO,
                            case when A.REMARK IS NULL or A.REMARK = '' then '' else A.REMARK  end  REMARK,
                            case when A.REL_CD1 IS NULL or A.REL_CD1 = '' then '' else A.REL_CD1  end  REL_CD1,
                            case when A.REL_CD2 IS NULL or A.REL_CD2 = '' then '' else A.REL_CD2  end  REL_CD2,
                            case when A.REL_CD3 IS NULL or A.REL_CD3 = '' then '' else A.REL_CD3  end  REL_CD3,
                            case when A.REL_CD4 IS NULL or A.REL_CD4 = '' then '' else A.REL_CD4  end  REL_CD4,
                            case when A.REL_CD5 IS NULL or A.REL_CD5 = '' then '' else A.REL_CD5  end  REL_CD5,  
							CASE WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
							A.Status,
							A.CreatedBY,
                           B.UserDispName as CreatedUser,
						   DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
							A.UpdatedBy,
                            C.UserDispName as UpdatedUser,
							DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
				FROM  `commoncode`  as A
								Left outer Join usermaster as B ON
											A.CreatedBY = B.UserID
								 Left outer Join  usermaster as C ON
											A.CreatedBY = C.UserID 
                    Where 	 	A.MAJOR_CD =MAJOR_CDs
						    AND A.MINOR_CD != '*****'   
                        order by  CAST(sort_No AS SIGNED);
                  END IF; 
                  
                  
                   IF SPstatus='3' THEN                   
                  
                  IF EXISTS(SELECT 1 FROM  `commoncode` WHERE COMP_CD = COMP_CDs AND MAJOR_CD =MAJOR_CDs AND MINOR_CD=MINOR_CDs) THEN
				 
                    UPDATE  `commoncode`
								SET  
                                `CD_NM`=CD_NMs,
                                 `SORT_NO`=SORT_NOs,
                                  `DEFAULT_FLAG`=DEFAULT_FLAGs,
                                  `REMARK`=REMARKs,
								`REL_CD1` = REL_CD1s,
                                `REL_CD2` = REL_CD2s,
                                `REL_CD3` = REL_CD3s,
                                `REL_CD4` = REL_CD4s,
                                `REL_CD5` = REL_CD5s,
								`IsApporved` = IsApporveds,
								`Status` = IsApporveds, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP
								WHERE COMP_CD = COMP_CDs AND MAJOR_CD =MAJOR_CDs AND MINOR_CD=MINOR_CDs;
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE;

        ELSE        
						INSERT INTO  `commoncode`
											(`COMP_CD`,
											`MAJOR_CD`,
											`MINOR_CD`,
											`CD_NM`,
											`DEFAULT_FLAG`,
											`SORT_NO`,
											`REMARK`,
											`REL_CD1`,
											`REL_CD2`,
											`REL_CD3`,
											`REL_CD4`,
											`REL_CD5`,
											`IsApporved`,
											`Status`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(COMP_CDs,
											MAJOR_CDs,
											MINOR_CDs,
											CD_NMs,
											DEFAULT_FLAGs,
											SORT_NOs,
											REMARKs,
											REL_CD1s,
											REL_CD2s,
											REL_CD3s,
											REL_CD4s,
											REL_CD5s,
											IsApporveds,
											Statuss,
											UpdatedBys,
											UpdatedBys); 

						Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE;

        END IF;
                  
                    END IF;
                
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_currrentTaskGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_currrentTaskGetPost`(  IN CurrentTaskIDs varchar(20),
IN taskids varchar(20), 
IN robotURLs varchar(50),
IN simulayoutcdss varchar(50),
IN startnodess varchar(60) ,
IN endnodess varchar(60) ,
IN currentnodess varchar(60) ,
IN curtaskids varchar(60) ,
IN curtaskdtlids varchar(60) ,
IN tasktypes varchar(60) ,
IN start_stations varchar(60) ,
IN final_stations varchar(60) ,
IN orderstatuss varchar(60) ,
IN workstatuss  varchar(60) ,
IN extraserviceruns  varchar(60) ,
IN column1s varchar(45),
IN column2s varchar(45),
IN column3s varchar(45),
IN column4s varchar(45),
IN column5s varchar(45),
IN SPstatus varchar(10)  
)
BEGIN		 

				IF SPstatus='1' THEN 
							SELECT  		CAST(A.CurrentTaskID as char(25)) as currenttaskid,
											CAST(A.taskid as char(25)) as taskid, 
                                            CASE   WHEN A.taskid='0' THEN "H1" Else IFNULL(B.workname,"H1")  END as workname,
                                              IFNULL(B.trayrack,"") as  trayrack, 
											 A.robotURL,
											 A.simulayoutcds,
											 A.startnodes,
											 A.endnodes,
											 A.currentnodes, 
											 A.curtaskid,
											 A.curtaskdtlid,
											 IFNULL(A.tasktype,"") as tasktype,
                                             A.start_station,
											 A.final_station,
											 A.orderstatus,
											 A.workstatus,
                                             A.extraservicerun,
											 IFNULL(A.column1,"") as  column1,
											 IFNULL(A.column2,"") as  column2,
											 IFNULL(A.column3,"") as  column3,
											 IFNULL(A.column4,"") as  column4,
											 IFNULL(A.column5,"") as  column5 
							 FROM 	currtasksetting  as A
									LEFT Outer JOIN taskchain as B 
											   ON A.taskid = B.taskid 
                           ; /*  Where 	
							 	 * A.robotURL like CONCAT('%', robotURLs, '%')   */


                  END IF; 
                  
                   IF SPstatus='2' THEN                   
                  
                  IF EXISTS(SELECT * FROM  currtasksetting  Where robotURL = robotURLs) THEN
				 
                    UPDATE  `currtasksetting`
								SET  
                                   `robotURL`=robotURLs,
                                   `simulayoutcds`=simulayoutcdss,
                                   `startnodes`=startnodess,
                                   `endnodes`=endnodess,
                                    currentnodes=currentnodess, 
									curtaskid=curtaskids,
									curtaskdtlid=curtaskdtlids,
									tasktype=curtaskdtlids,
                                    start_station = start_stations,
                                    final_station=final_stations,
                                    orderstatus=orderstatuss,
                                    workstatus=workstatuss,
                                    extraservicerun = extraserviceruns,
                                    `column1` =  column1s ,
									`column2` =  column2s ,
									`column3` =  column3s ,
									`column4` =  column4s ,
									`column5` =  column5s   
                               Where robotURL = robotURLs ;
                                  
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE;

        ELSE        
						INSERT INTO  `currtasksetting`
											(`robotURL`,
											`simulayoutcds`,
											`startnodes`,
											`endnodes`,
                                             currentnodes, 
											curtaskid,
											curtaskdtlid,
											tasktype,
                                            start_station,
                                            final_station,
											orderstatus,
											workstatus,
                                            extraservicerun,
                                            column1,
											column2,
											column3,
											column4,
											column5)
											VALUES
											(robotURLs,
											simulayoutcdss,
											startnodess,
											endnodess,
                                            currentnodess, 
											curtaskids,
											curtaskdtlids,
											tasktypes,
                                            start_stations,
                                            final_stations,
											orderstatuss,
											workstatuss,
                                            extraserviceruns,
                                            column1s,
											column2s,
											column3s,
											column4s,
											column5s); 

						Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE;

         END IF;
       END IF;
       
       
       IF SPstatus='3' THEN 
							SELECT  		CAST(A.CurrentTaskID as char(25)) as currenttaskid,
											CAST(A.taskid as char(25)) as taskid, 
                                             B.workname,
                                             B.trayrack, 
											 A.robotURL,
											 A.simulayoutcds,
											 A.startnodes,
											 A.endnodes,
											 A.currentnodes, 
											 A.curtaskid,
											 A.curtaskdtlid,
											 A.tasktype,
                                             A.start_station,
											 A.final_station,
											 A.orderstatus,
											 A.workstatus,
                                             A.extraservicerun,
											 IFNULL(A.column1,"") as  column1,
											 IFNULL(A.column2,"") as  column2,
											 IFNULL(A.column3,"") as  column3,
											 IFNULL(A.column4,"") as  column4,
											 IFNULL(A.column5,"") as  column5 
							 FROM 	currtasksetting  as A
									LEFT Outer JOIN taskchain as B 
											   ON A.taskid = B.taskid 
                             Where 	
									robotURL like CONCAT('%', robotURLs, '%')   ;

                  END IF; 
                  
                   /*   Rack Status update     */
                   
                   
                     IF SPstatus='4' THEN                   
								UPDATE  `currtasksetting`
									SET  
                                    `column1` =  column1s  
                                 Where taskid = taskids ;
                                 
                                 UPDATE  taskchain 
											SET  
											`trayrack` =  column1s  
								 WHERE taskid = taskids;
                        	Select 'U101' as CODE,'okSaved' as MSG ,'Tray Rack Updated' as VALUE;
                  END IF; 
                  
                    /*   Extra Run service run ok or not 0 for not run and 1 for run     */
                    IF SPstatus='5' THEN      
                    
								UPDATE  `currtasksetting`
									SET  
                                    `extraservicerun` =  extraserviceruns 
                                     Where robotURL = robotURLs ;
                                 
                                 UPDATE  taskchain 
											SET  
											`taskrunok` =  extraserviceruns  
								 WHERE			   robotip = robotURLs 
											AND  orderstatus =1;
                                            
                        	Select 'U101' as CODE,'okSaved' as MSG ,'Tray Rack Updated' as VALUE;
                  END IF; 
                  
                    /*  Old one Extra Run service run ok or not 0 for not run and 1 for run    
                    IF SPstatus='5' THEN      
                    
								UPDATE  `currtasksetting`
									SET  
                                    `extraservicerun` =  extraserviceruns 
                                     Where robotURL = robotURLs ;
                                 
                                 UPDATE  taskchain 
											SET  
											`taskrunok` =  extraserviceruns  
								 WHERE			   robotip = robotURLs 
											AND tasktype >2
										    AND    orderstatus <=5;
                                            
                        	Select 'U101' as CODE,'okSaved' as MSG ,'Tray Rack Updated' as VALUE;
                  END IF; 
                 */
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_GetCodeValues` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_GetCodeValues`( 
IN Codes varchar(50),
IN Valuess varchar(50), 
IN SPstatus varchar(10)  
)
BEGIN          
				IF SPstatus='COMM' THEN 
						SELECT ' ' as codes, ' ' as cdnames
						UNION ALL
						SELECT 	 A.MINOR_CD as codes,
									 A.CD_NM as cdnames
								/*   ,'' as Value2,
								     '' as Value3 */
							FROM  `commoncode` as A
							Where 	 	 A.MAJOR_CD=Codes
									AND  A.MINOR_CD !='*****'
                                    AND  A.IsApporved='1' ;
                                 --   order by  CAST(sort_No AS SIGNED);
                     END IF;  
                     
                     
                     IF SPstatus='COMM1' THEN 
								SELECT    ' ' as dropdownval
								UNION ALL
								SELECT 	  A.CD_NM as dropdownval
									FROM  `commoncode` as A
									Where 	 	 A.MAJOR_CD=Codes
											AND  A.MINOR_CD !='*****'
											AND  A.IsApporved='1' ;
                                 --   order by  CAST(sort_No AS SIGNED);
                     END IF;  
                     
                         IF SPstatus='COMM2' THEN 
								SELECT    ' ' as dropdownval
								UNION ALL
								SELECT 	   A.robotname as dropdownval
										FROM  `robotnames` as A
										WHERE   A.IsApporved='1' ; 
                     END IF;  
                     
                     
                     IF SPstatus='SIMU1' THEN 
								SELECT Distinct startnode as dropdownval FROM  robotsimgraph 
								Where    	simulayoutcd=Codes
										AND IsApporved='1'
								ORDER BY CAST(startnode AS DOUBLE) ;
					 END IF;  
                     
                          IF SPstatus='SIMU2' THEN 
							SELECT    ' ' as dropdownval
								UNION ALL
								SELECT Distinct startnode as dropdownval FROM  robotsimgraph 
								Where    	simulayoutcd=Codes
										AND IsApporved='1'
								ORDER BY CAST(startnode AS DOUBLE) ;
					 END IF;  
                     
                         IF SPstatus='SIMU3' THEN 
								SELECT Distinct tablearuco as dropdownval FROM  robotsimtablegraph 
								Where    	simulayoutcd=Codes
										AND IsApporved='1'
                                        AND tablearuco Like  CONCAT('%', Valuess, '%')   
								ORDER BY CAST( SUBSTRING(tablearuco, 2) AS DOUBLE) ,CAST( endnode AS DOUBLE);
					 END IF;  
                     
                        IF SPstatus='SIMU4' THEN 
								SELECT Distinct custtableno as codes , tablearuco as cdnames FROM  robotsimtablegraph 
								Where    	simulayoutcd=Codes
										AND IsApporved='1'
                                        AND tablearuco Like  CONCAT('%', Valuess, '%')   
								ORDER BY CAST( SUBSTRING(tablearuco, 2) AS DOUBLE) ;
					 END IF;     
                 /*     
                     IF SPstatus='COMM1' THEN 
						  SELECT 	 '' as cdes,
									 '' as value1,
								     '' as value2,
								     '' as value3 
						 
                     END IF;  
                     */
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_GetSetMapJson` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_GetSetMapJson`(IN templatecodes varchar(15),
IN mapjsons  longtext,
IN SPstatus varchar(10) )
BEGIN
		IF SPstatus='1' THEN 
				SELECT      A.mapjson 
				FROM  mapjson as A   
				Where 		 A.templatecode = templatecodes ;
		 END IF;
         
         	  IF SPstatus='2' THEN 

					 IF EXISTS(SELECT 1 FROM mapjson WHERE templatecode = templatecodes  ) THEN 
                                      
												UPDATE mapjson 
														SET 
														 mapjson = mapjsons 
														WHERE templatecode = templatecodes; 
												
											Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
							ELSE   
											INSERT INTO mapjson
														( `templatecode`,  
                                                        `mapjson`)
														VALUES
														(
														templatecodes,
                                                        mapjsons); 

											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF; 
				END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_MenuGet` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_MenuGet`(IN RoleIDs varchar(10))
BEGIN
		SELECT 	Parent_MenuID,
				MenuID,
				MenuKRNM,
				MenuENNM,
				MenuURL,
				menuClass
				FROM  menumaster
		where
		RoleID = RoleIDs
		and IsApporved='1'
		and Status='1';
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_MenuGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_MenuGetPost`(IN MenuIdentitys INT, 
IN MenuIDs varchar(45),
IN MenuNames varchar(100),
IN Parent_MenuIDs varchar(45),
IN RoleIDs INT,
IN MenuKRNMs varchar(45),
IN MenuENNMs varchar(45),
IN MenuURLs varchar(100), 
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  
)
BEGIN
				IF SPstatus='1' THEN
							SELECT '0' as RowsVal,
									A.MenuIdentity, 
									A.MenuID ,
									A.MenuName ,
									A.Parent_MenuID as ParentMenuID,
									A.RoleID,
									D.RoleName,
									A.MenuKRNM,
									A.MenuENNM,
									A.MenuURL,
									CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
									A.CreatedBY,
                                    B.UserDispName as CreatedUser,
									 DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
									A.UpdatedBy,
                                    C.UserDispName as UpdatedUser,
									 DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate,
									A.Status
				FROM `menumaster` as A
				 Left outer Join bconnectpoc.usermaster as B ON
											A.CreatedBY = B.UserID
								 Left outer Join bconnectpoc.usermaster as C ON
											A.CreatedBY = C.UserID
				LEFT OUTER JOIN `rolemaster` as D
					ON A.RoleID = D.RoleID
                    Where 	 A.MenuName like CONCAT('%', MenuNames, '%') 
						AND  A.MenuENNM like CONCAT('%', MenuENNMs, '%') ;
                        
                  END IF;
                  
                  IF SPstatus='2' THEN
                  IF EXISTS(SELECT 1 FROM `menumaster` WHERE MenuID = MenuIDs AND MenuName = MenuNames AND Parent_MenuID = Parent_MenuIDs AND RoleID=RoleIDs) THEN
					UPDATE `bconnectpoc`.`menumaster`
								SET
								/*-- `MenuIdentity` = MenuIdentitys,
								-- `MenuID` = MenuIDs,
								-- `MenuName` = MenuNames,
								-- `Parent_MenuID` = Parent_MenuIDs,
								-- `RoleID` = RoleIDs,*/
								`MenuKRNM` = MenuKRNMs,
								`MenuENNM` = MenuENNMs,
								`MenuURL` = MenuURLs,
								`IsApporved` = IsApporveds,
								`Status` = IsApporveds, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP,
								`menuClass` = CASE WHEN Parent_MenuIDs='*' THEN 'nav-link active' Else 'nav-link' END
								WHERE MenuID = MenuIDs AND MenuName = MenuNames AND Parent_MenuID = Parent_MenuIDs AND RoleID=RoleIDs;
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'Menu data updated' as VALUE;

        ELSE
						INSERT INTO `menumaster`
										( `MenuID`,
										`MenuName`,
										`Parent_MenuID`,
										`RoleID`,
										`MenuKRNM`,
										`MenuENNM`,
										`MenuURL`,
										`IsApporved`,
										`Status`,
										`CreatedBY`,
										`UpdatedBy`,
										`menuClass`)
										VALUES
										( MenuIDs,
										MenuNames,
										Parent_MenuIDs,
										RoleIDs,
										MenuKRNMs,
										MenuENNMs,
										MenuURLs,
										IsApporveds,
										Statuss,
										UpdatedBys,
										UpdatedBys,
										CASE WHEN Parent_MenuIDs='*' THEN 'nav-link active' Else 'nav-link' END);

						Select 'I101' as CODE,'okSaved' as MSG ,'Roles data inserted' as VALUE;

        END IF;
                  
                    END IF;
                
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotAlaramMessageGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotAlaramMessageGetPost`(
IN startDates VARCHAR(20),
IN endDates VARCHAR(20), 
IN ipaddresss  varchar(45), 
IN alarmcodes varchar(45), 
IN alarmmsgs  varchar(45), 
IN commentss  varchar(45), 
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  )
BEGIN 
DECLARE  robotIDs varchar(20) ; 

					IF SPstatus='1' THEN 
								SELECT 	   '0' as rowsVal,
												A.ipaddress, 
												substr(A.alarmcode, 1,1) as alarmType,
												A.alarmcode, 
                                                A.alarmmsg, 
                                                A.comments,
                                               CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d %H:%i:%S') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d %H:%i:%S') AS UpdatedDate
									FROM  robotalarmmaster as A 
											Left outer Join usermaster as B ON
																	A.CreatedBY = B.UserID
														 Left outer Join usermaster as C ON
																	A.UpdatedBy = C.UserID  
									 WHERE 		 A.alarmcode like CONCAT('%', alarmcodes, '%')  
											AND	 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') BETWEEN startDates AND endDates ; 
									  END IF;
                                      
                                      
                                  IF SPstatus='3' THEN 
										 SELECT 	     '0' as rowsVal,
												A.ipaddress, 
												substr(A.alarmcode, 1,1) as alarmType,
												A.alarmcode, 
                                                A.alarmmsg, 
                                                A.comments,
                                               CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d %H:%i:%S') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d %H:%i:%S') AS UpdatedDate
									FROM  robotalarmmaster as A 
											Left outer Join usermaster as B ON
																	A.CreatedBY = B.UserID
														 Left outer Join usermaster as C ON
																	A.UpdatedBy = C.UserID  
											 Where 	A.Status=1;
									  END IF;
									  
									  IF SPstatus='2' THEN  
										INSERT INTO robotalarmmaster
															(`ipaddress`,
                                                            `alarmcode`,
															`alarmmsg`,
                                                            `comments`,
															`IsApporved`,
															`Status`,
															`CreatedBY`,
															`UpdatedBy`)
															VALUES
															(ipaddresss,
                                                            alarmcodes,
															alarmmsgs,
                                                            commentss,
															IsApporveds,
															Statuss,
															UpdatedBys,
															UpdatedBys);  
											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF;   
                             
                           IF SPstatus='4' THEN  
										  Update robotalarmmaster SET Status=0 
										  Where Status>=0;
										  
										  Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE;  
							END IF;  
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotFileUpload` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotFileUpload`(  
IN  fids int,
IN  filenames varchar(150),
IN  filepaths varchar(250),
IN  cols1s varchar(45),  
IN  SPstatus varchar(10)  )
BEGIN 

					IF SPstatus='1' THEN 
								SELECT		fid,
											filename,
											filepath,
											cols1 
								FROM  fileuploads;
				     END IF;
                                      
					 IF SPstatus='2' THEN  
                   
								INSERT INTO  `fileuploads`
											(`filename`,
											`filepath`,
											`cols1` )
											VALUES
											( filenames,
											  filepaths,
											  cols1s ); 

							Select 'I101' as CODE,'okSaved' as MSG ,'Inserted' as VALUE; 
               END IF; 
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotGraphGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotGraphGetPost`(IN graphids INT,  
IN startnodes  varchar(45),
IN endnodes varchar(45),
IN distances DECIMAL(10,2),
IN reachtimes DECIMAL(10,2),
IN speeds DECIMAL(10,2),
IN iconxvals DECIMAL(10,2),
IN iconyvals DECIMAL(10,2),
IN linexvals DECIMAL(10,2),
IN lineyvals DECIMAL(10,2),
IN commentss VARCHAR(100), 
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  )
BEGIN 
					IF SPstatus='1' THEN 
									SELECT 	   '0' as rowsVal,
											    A.graphid,
												A.startnode,
												A.endnode,
												A.distance,
												A.reachtime,
												A.speed,
												A.iconxval,
												A.iconyval,
												A.linexval,
												A.lineyval,
												A.comments, 
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotsimgraph as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID  
										Where 	 A.startnode like CONCAT('%',startnodes, '%') 
											ORDER BY CAST( A.startnode AS DOUBLE) ,CAST( A.endnode AS DOUBLE);
									  END IF;
									  
									  IF SPstatus='2' THEN 

									  IF EXISTS(SELECT 1 FROM robotsimgraph WHERE graphid = graphids ) THEN 
                                      
												UPDATE robotsimgraph 
														SET 
														 startnode = startnodes,
														 endnode = endnodes,
														 distance = distances,
														 reachtime = reachtimes,
														 speed = speeds,
														 iconxval = iconxvals,
														 iconyval = iconyvals,
														 linexval = linexvals,
														 lineyval = lineyvals,
														 comments  = commentss, 
														 IsApporved = IsApporveds,
													     Status  = IsApporveds, 
														 UpdatedBy  = UpdatedBys,
														 UpdatedDate  = CURRENT_TIMESTAMP
														WHERE  graphid = graphids; 
												
											Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
							ELSE   
											INSERT INTO robotsimgraph
														(`startnode`,
														`endnode`,
														`distance`,
														`reachtime`,
														`speed`,
														`iconxval`,
														`iconyval`,
														`linexval`,
														`lineyval`,
														`comments`, 
														`IsApporved`,
														`Status`,
														`CreatedBY`,
														`UpdatedBy`)
														VALUES
														(
														startnodes,
														endnodes,
														distances,
														reachtimes,
														speeds,
														iconxvals,
														iconyvals,
														linexvals,
														lineyvals,
														commentss, 
														IsApporveds,
														Statuss,
														UpdatedBys,
														UpdatedBys); 

											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF; 
										END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotMasterGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotMasterGetPost`(IN robotmIDs INT,  
IN robotnames  varchar(45),
IN robotmodelnos varchar(45),
IN robotmipaddresss varchar(45),
IN robotmhostnms varchar(45),
IN robotsipaddresss varchar(45),
IN robotshostnms varchar(45),
IN minChargeValues INT,  
IN maxChargeValues INT,  
IN chkchargectatuss varchar(45),
IN Commentss VARCHAR(100),
IN burl1s varchar(100),
IN burl2s varchar(100),
IN burl3s varchar(100),
IN burl4s varchar(100),
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  )
BEGIN

DECLARE  robotIDs varchar(20) ; 

DECLARE robotmodelnogen varchar(20) ; 

					IF SPstatus='1' THEN 
									SELECT 	   '0' as rowsVal,
												A.robotmID,
												A.robotID,
												D.robotname,
												A.robotmodelno,
												A.robotmipaddress,
												A.robotmhostnm,
												A.robotsipaddress,
												A.robotshostnm,
												A.minChargeValue,
												A.maxChargeValue,
                                                CASE   WHEN A.chkchargectatus='1' THEN 'true' Else 'false' END as chkchargectatus ,
												A.Comments,
												A.burl1,
												A.burl2,
												A.burl3,
												A.burl4,
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotmaster as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID 
													 LEFT OUTER JOIN robotnames as D  ON
																		A.robotID=  D.robotID  
										Where 	 A.robotmodelno like CONCAT('%', robotmodelnos, '%')  ;
											
									  END IF;
									  
									  IF SPstatus='2' THEN 

									  IF EXISTS(SELECT 1 FROM robotmaster WHERE robotmID = robotmIDs ) THEN 
                                      
												UPDATE robotmaster 
														SET 
														 robotmipaddress = robotmipaddresss,
														 robotmhostnm = robotmhostnms,
														 robotsipaddress = robotsipaddresss,
														 robotshostnm = robotshostnms,
														 minChargeValue = minChargeValues,
														 maxChargeValue = maxChargeValues,
														 chkchargectatus = chkchargectatuss,
														 Comments = Commentss,
														 burl1 = burl1s,
														 burl2  = burl2s,
														 burl3 = burl3s,
														 burl4 = burl4s,
														 IsApporved = IsApporveds,
													     Status  = IsApporveds, 
														 UpdatedBy  = UpdatedBys,
														 UpdatedDate  = CURRENT_TIMESTAMP
												 WHERE  robotmID  = robotmIDs;  
                                                        
                                                        
                                                          UPDATE  `currtasksetting`
															SET  
															   `robotURL`=robotmipaddresss;
												
											Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
							ELSE        
                              SET robotIDs =  (Select DISTINCT robotID from  robotnames as D  
																	Where D.robotname= robotnames) ;
                                                                    
								 SET robotmodelnogen = Concat(robotmodelnos , "_",(Select count(*) +1 from robotmaster ) );
                             
											INSERT INTO robotmaster
														   (`robotID`,
														    `robotmodelno`,
														    `robotmipaddress`,
														    `robotmhostnm`,
														    `robotsipaddress`,
														    `robotshostnm`,
														    `minChargeValue`,
														    `maxChargeValue`,
														    `chkchargectatus`,
														    `Comments`,
														    `burl1`,
														    `burl2`,
														    `burl3`,
														    `burl4`,
														    `IsApporved`,
														    `Status`,
														    `CreatedBY`,
														    `UpdatedBy`)
														VALUES
														    (robotIDs,
														     robotmodelnogen,
														     robotmipaddresss,
														     robotmhostnms,
														     robotsipaddresss,
														     robotshostnms,
														     minChargeValues,
														     maxChargeValues,
														     chkchargectatuss,
														     Commentss,
														     burl1s,
														     burl2s,
														     burl3s,
														     burl4s,
														     IsApporveds,
														     Statuss,
														     UpdatedBys,
														     UpdatedBys); 
                                                        
                                                        INSERT INTO  `currtasksetting`
															  (`robotURL`,
															  `simulayoutcds`,
															  `startnodes`,
															  `endnodes`,
															   currentnodes, 
															   curtaskid,
															   curtaskdtlid,
															   tasktype,
															   start_station,
															   final_station,
															   orderstatus,
															   workstatus,
															   extraservicerun,
															   column1,
															   column2,
															   column3,
															   column4,
															   column5)
															VALUES
															   (robotmipaddresss,
															    '',
															    '',
															    '',
															    '', 
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    '',
															    ''); 

											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF; 
										END IF;
                                        
                                        
                         IF SPstatus='3' THEN 
													
									SELECT 	   '0' as rowsVal,
												A.robotmID,
												A.robotID,
												D.robotname,
												A.robotmodelno,
												A.robotmipaddress,
												A.robotmhostnm,
												A.robotsipaddress,
												A.robotshostnm,
												A.minChargeValue,
												A.maxChargeValue,
                                                CASE   WHEN A.chkchargectatus='1' THEN 'true' Else 'false' END as chkchargectatus ,
												A.Comments,
												A.burl1,
												A.burl2,
												A.burl3,
												A.burl4,
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotmaster as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID 
													 LEFT OUTER JOIN robotnames as D  ON
																		A.robotID=  D.robotID   
                                                                        ORDER BY robotmID 
                                                                        LIMIT 1;
                      END IF;
                      
                      
                        IF SPstatus='4' THEN 
													
									SELECT 	   '0' as rowsVal,
												A.robotmID,
												A.robotID,
												D.robotname,
												A.robotmodelno,
												A.robotmipaddress,
												A.robotmhostnm,
												A.robotsipaddress,
												A.robotshostnm,
												A.minChargeValue,
												A.maxChargeValue,
                                                CASE   WHEN A.chkchargectatus='1' THEN 'true' Else 'false' END as chkchargectatus ,
												A.Comments,
												A.burl1,
												A.burl2,
												A.burl3,
												A.burl4,
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotmaster as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID 
													 LEFT OUTER JOIN robotnames as D  ON
																		A.robotID=  D.robotID   
                                                                        ORDER BY robotmID ; 
                      END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotNameGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotNameGetPost`(IN robotids INT,  
IN robotnamess varchar(100),
IN robottypes varchar(20),
IN Commentss varchar(200), 
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  )
BEGIN

DECLARE  robotTypeCode varchar(20) ;
 

IF SPstatus='1' THEN 
                SELECT 		'0' as RowsVal,
							A.robotid,
							A.robotname,
                           D.CD_NM as robottype,
							 case when A.Comments IS NULL or A.Comments = ''
											then ''
											else A.Comments
									   end  Comments,
							CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
							A.Status,
							A.CreatedBY,
                           B.UserDispName as CreatedUser,
						   DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
							A.UpdatedBy,
                            C.UserDispName as UpdatedUser,
							DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
				FROM  robotnames  as A
								Left outer Join usermaster as B ON
											A.CreatedBY = B.UserID
								 Left outer Join usermaster as C ON
											A.CreatedBY = C.UserID 
								 LEFT OUTER JOIN commoncode as D  ON
													 D.MINOR_CD=  A.robottype
												AND  D.MINOR_CD !='*****'
												AND  D.Major_CD ='A002'
												AND  D.IsApporved='1'  
                    Where 	 A.robotname like CONCAT('%', robotnamess, '%')  ;
                        
                  END IF;
                  
                  IF SPstatus='2' THEN
                  
                   SET robotTypeCode =  (Select  MINOR_CD from  commoncode as D  
												Where D.CD_NM= robottypes
												AND  D.MINOR_CD !='*****'
												AND  D.Major_CD ='A002'
												AND  D.IsApporved='1'  ) ;

                  IF EXISTS(SELECT 1 FROM robotnames WHERE robotid = robotids ) THEN 
                    UPDATE robotnames
								SET  
                                `robottype`=robotTypeCode,
								`Comments` = Commentss,
								`IsApporved` = IsApporveds,
								`Status` = IsApporveds, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP
								WHERE `robotid` = robotids; 
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
        ELSE        
						INSERT INTO robotnames
												(`robotname`,
                                                `robottype`,
												`Comments`,
												`IsApporved`,
												`Status`,
												`CreatedBY`, 
												`UpdatedBy`)
												VALUES
												(
												robotnamess,
                                                robotTypeCode,
												Commentss,
												IsApporveds,
												Statuss,
												UpdatedBys,
												UpdatedBys); 

						Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
        END IF; 
                    END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotOrderMasterDetailExecuteMDGETPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotOrderMasterDetailExecuteMDGETPost`(  
IN taskids varchar(20),  
IN startDates  VARCHAR(20),
IN endDates VARCHAR(20),
IN orderexecmids VARCHAR(20), 
IN worknames  VARCHAR(100),
IN robotips  VARCHAR(30),
IN tasktypes  VARCHAR(30),
IN ordertypes  VARCHAR(20),
IN orderstatuss  VARCHAR(20),
IN commentss  VARCHAR(100),

IN orderexecdids VARCHAR(20), 
 
IN startworkstations  VARCHAR(20),
IN finalworkstations  VARCHAR(20),

IN initialstations  VARCHAR(20),
IN deliverytable  VARCHAR(20),

IN completestatuss  varchar(10),
IN workstatuss  varchar(10),
IN dtlcomments  varchar(200),
IN UpdatedBys VARCHAR(20),
IN SPstatus varchar(10)  
)
BEGIN	  
DECLARE  orderexecmidM varchar(10);

				IF SPstatus='1' THEN
								SELECT 		  ''   as rowsVal,  
												A.taskid,
												A.robotip,
												A.workname,	
												B.startworkstation,
												B.finalworkstation,
												D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.ordertype,
												B.completestatus ,
												A.orderstatus as workstatus,
                                                A.comments,
                                                CONVERT(A.createdDate,char) as createdDate,
												F.UserDispName as updatedBy,
												CONVERT(B.updatedDate,char) as  updatedDate
								 FROM  `robotorderexecmaster`  as A
														Left outer Join  robotorderexecdetail as B ON
																			A.orderexecmid = B.orderexecmid
														Left outer Join  robotmaster as C ON
																			A.robotip = C.robotmipaddress
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD='O3'
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD='OD1'
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
															Left Outer join usermaster as F On
																A.UpdatedBy = F.UserId     
								Where 	 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') =  DATE_FORMAT(CURRENT_TIMESTAMP, '%Y-%m-%d')
										Order by   convert(B.updatedDate , datetime)  DESC , Cast(B.orderexecdid as double) Desc, Cast(B.startworkstation as double) Desc, Cast(B.finalworkstation as double) ;
					END IF;
                    
                    
                    IF SPstatus='2' THEN
								SELECT 		  ''   as rowsVal,  
												A.robotip,
                                                A.taskid,
												A.workname,	
												B.startworkstation,
												B.finalworkstation,
												D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.ordertype,
												B.completestatus ,
												A.orderstatus as workstatus,
                                                A.comments,
                                                CONVERT(A.createdDate,char) as createdDate,
												F.UserDispName as updatedBy,
												CONVERT(B.updatedDate,char) as  updatedDate
								 FROM  `robotorderexecmaster`  as A
														Left outer Join  robotorderexecdetail as B ON
																			A.orderexecmid = B.orderexecmid
														Left outer Join  robotmaster as C ON
																			A.robotip = C.robotmipaddress
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD='O3'
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD='OD1'
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
															Left Outer join usermaster as F On
																A.UpdatedBy = F.UserId     
									Where 			 A.workname like CONCAT('%', worknames, '%')   
											AND		 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') BETWEEN startDates AND endDates
										Order by   convert(B.updatedDate , datetime)  DESC , Cast(B.orderexecdid as double) Desc, Cast(B.startworkstation as double) Desc, Cast(B.finalworkstation as double) ;
				
					END IF;
                    
                    
          IF SPstatus='3' THEN  
          
                  IF EXISTS(SELECT 1 FROM robotorderexecmaster WHERE orderexecmid = orderexecmids ) THEN
				                 
                    UPDATE  `robotorderexecmaster`
								SET  
                                tasktype=tasktypes,
                                ordertype=ordertypes,
                                `orderstatus`=orderstatuss, 
                                 `comments`=commentss, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP
								WHERE orderexecmid = orderexecmids;
                                
                                   INSERT INTO `robotorderexecdetail`
											(`orderexecdid`, 
                                            `orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecdids,  
                                            orderexecmids,
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);  
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;

        ELSE     
						INSERT INTO `robotorderexecmaster`
										(`orderexecmid`,
                                        `taskid`,
                                        `robotip`,
										`workname`,										
                                        `tasktype`,
										`ordertype`,
										`orderstatus`,
										`comments`,
										`CreatedBY`, 
										`UpdatedBy`)
										VALUES
										(orderexecmids,
                                        taskids,
                                        robotips,
										worknames,										
                                        tasktypes,
										ordertypes,
										orderstatuss,
										commentss,
										UpdatedBys,
										UpdatedBys);
                                        
                           INSERT INTO `robotorderexecdetail`
											(`orderexecdid`, 
                                            `orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecdids,  
                                            orderexecmids,
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);
                                            
                                     UPDATE  currtasksetting 
											 SET 
												    `robotURL` = robotips,  
													`curtaskid` = orderexecmids,
													`curtaskdtlid` = orderexecdids,
													 `tasktype` = tasktypes ,
                                                      `taskid` = taskids 
												where  `robotURL` = robotips;

						Select 'I101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;

        END IF; 
                    END IF;      
                    
                    
             IF SPstatus='4' THEN   
						UPDATE  `robotorderexecmaster`
								SET   
                                `orderstatus`=orderstatuss  ,
                                `UpdatedDate` = CURRENT_TIMESTAMP ,
                                 `comments` = commentss
						 WHERE orderexecmid = orderexecmids;
                                
                         UPDATE  `robotorderexecdetail`
								SET      
                                            `completestatus`=completestatuss,
                                            `workstatus`=workstatuss , 
											 `comments` = commentss ,
                                            `UpdatedDate` = CURRENT_TIMESTAMP
						  WHERE orderexecmid = orderexecmids
								AND orderexecdid=orderexecdids;        
                                            
                             UPDATE  currtasksetting 
									  SET  	  `taskid` = taskids,
											  `startnodes` = startworkstations,
											  `endnodes` = finalworkstations,
											   `currentnodes` = dtlcomments,
											   `curtaskid` = orderexecmids,
											   `curtaskdtlid` = orderexecdids, 
                                               `workstatus`= workstatuss,
                                               `orderstatus` = orderstatuss,
                                               `start_station` = initialstations,
                                               `final_station` = deliverytable 
                                                WHERE `robotURL` = robotips;
                                                   
								UPDATE `bconnectpoc`.`taskchain`
											SET 
												   `orderstatus` = completestatuss, 
                                                    `startworkstation` = initialstations,
												    `endworkstation` = deliverytable,
                                                     `comments` = commentss ,
													 `UpdatedDate` = CURRENT_TIMESTAMP
											  WHERE	 `taskid` = taskids;
						Select 'I101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE; 
                  
                    END IF;      
                    
                    IF SPstatus='5' THEN   
                                            
                             UPDATE  currtasksetting SET `taskid` = taskids,`currentnodes`=startworkstations   ,startnodes=startworkstations
								where `robotURL` = robotips;
                             
							Select 'I101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE; 
                        
                    END IF;   
                     
                    IF SPstatus='6' THEN
								
                                SELECT 	 		A.workname,	
												B.startworkstation as startstation,
												B.finalworkstation as endstation, 
												A.ordertype,
												B.completestatus as taskrunstatus,
												A.orderstatus as completedstatus , 
                                               (
													CASE 
														WHEN D.graphid IS NULL
														THEN     'N'
														ELSE 	 'T'
													END
												  ) as tablenodes , 
                                                (
													CASE 
														WHEN D.graphid IS NULL
														THEN     C.iconxval
														ELSE D.iconxval-20 /* -1 */
													END
												  ) as startxval , 
												 (
													CASE 
														WHEN D.graphid IS NULL
														THEN    C.iconyval
														ELSE   D.iconyval-20 /* -1 */ -1
													END
												  ) as startyval,
                                                  (
													CASE 
														WHEN D.graphid IS NULL
														THEN   C.linexval
														ELSE   D.iconxval
													END
												  ) as endxval , 
												  (
													CASE 
														WHEN D.graphid IS NULL
														THEN   C.lineyval
														ELSE   D.iconyval
													END
												  ) as endyval , 
                                                  (
													CASE 
														WHEN D.graphid IS NULL
														THEN   C.directions
														ELSE   D.directions
													END
												  ) as directions
								 FROM  robotorderexecmaster as A
														Left Outer Join  robotorderexecdetail as B
																	 ON	A.orderexecmid = B.orderexecmid 
														Left Outer Join robotsimgraph as C
																	  ON B.startworkstation = C.startnode
                                                                      AND B.finalworkstation = C.endnode
                                                                       AND C.simulayoutcd = "1001"
														Left Outer Join robotsimtablegraph as D
																		  ON B.startworkstation = D.endnode
                                                                      AND B.finalworkstation = D.tablearuco
                                                                       AND D.simulayoutcd = "1001"
								 Where A.orderexecmid = orderexecmids 
                                     ORDER BY CAST(B.orderexecdid as double) ; 
                    END IF; 
                    
                       IF SPstatus='7' THEN  
          
                  IF EXISTS(SELECT 1 FROM robotorderexecmaster WHERE orderexecmid = orderexecmids ) THEN
				                 
                    UPDATE  `robotorderexecmaster`
								SET  
                                tasktype=tasktypes,
                                ordertype=ordertypes,
                                `orderstatus`=orderstatuss, 
                                 `comments`=commentss, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP
								WHERE orderexecmid = orderexecmids;
                                
                                   INSERT INTO `robotorderexecdetail`
											(`orderexecdid`, 
                                            `orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecdids,  
                                            orderexecmids,
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);  
                                            
                                      UPDATE taskchain 
											SET 
													`startworkstation` = dtlcomments,
													`endworkstation` =startworkstations
											WHERE	`taskid` = taskids;
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;

        ELSE     
						INSERT INTO `robotorderexecmaster`
										(`orderexecmid`,
                                        `taskid`,
                                        `robotip`,
										`workname`,										
                                        `tasktype`,
										`ordertype`,
										`orderstatus`,
										`comments`,
										`CreatedBY`, 
										`UpdatedBy`)
										VALUES
										(orderexecmids,
                                        taskids,
                                        robotips,
										worknames,										
                                        tasktypes,
										ordertypes,
										orderstatuss,
										commentss,
										UpdatedBys,
										UpdatedBys);
                                        
                           INSERT INTO `robotorderexecdetail`
											(`orderexecdid`, 
                                            `orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecdids,  
                                            orderexecmids,
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);
                                            
                                     UPDATE  currtasksetting 
											 SET 
												   `robotURL` = robotips,  
													`curtaskid` = orderexecmids,
													`curtaskdtlid` = orderexecdids,
													`tasktype` = tasktypes ,
                                                     `taskid` = taskids;
                                                     
								   UPDATE taskchain 
											SET 
													`startworkstation` = dtlcomments,
													`endworkstation` =startworkstations
											WHERE	`taskid` = taskids; 

						Select 'I101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;

        END IF; 
                    END IF;      
            
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotSimulationGraphGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotSimulationGraphGetPost`(IN graphids INT,  
IN simulayoutcds  varchar(15),
IN startnodes  varchar(45),
IN endnodes varchar(45),
IN distances DECIMAL(10,2),
IN reachtimes DECIMAL(10,2),
IN speeds DECIMAL(10,2),
IN iconxvals DECIMAL(10,2),
IN iconyvals DECIMAL(10,2),
IN linexvals DECIMAL(10,2),
IN lineyvals DECIMAL(10,2),
IN directionss varchar(45),
IN rail_types  varchar(45),
IN nodetypes  varchar(45),
IN commentss VARCHAR(100), 
IN IsApporveds varchar(10),
IN UpdatedBys varchar(10),
IN Statuss  varchar(10),
IN SPstatus varchar(10)  )
BEGIN 
					IF SPstatus='1' THEN 
									SELECT 	   '0' as rowsVal,
											    A.graphid,
                                                A.simulayoutcd,
												A.startnode,
												A.endnode,
												A.distance,
												A.reachtime,
												A.speed,
												A.iconxval,
												A.iconyval,
												A.linexval,
												A.lineyval,
                                                A.directions,
                                                A.rail_type,
                                                A.nodetype,
												A.comments, 
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotsimgraph as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID  
										Where 		 A.simulayoutcd = simulayoutcds
												AND  A.startnode like CONCAT('%',startnodes, '%') 
											ORDER BY CAST( A.startnode AS DOUBLE) ,CAST( A.endnode AS DOUBLE);
									  END IF;
                                      
                                      
                                 IF SPstatus='3' THEN 
									SELECT 	   '0' as rowsVal,
											    A.graphid,
                                                A.simulayoutcd,
												A.startnode,
												A.endnode,
												A.distance,
												A.reachtime,
												A.speed,
												A.iconxval,
												A.iconyval,
												A.linexval,
												A.lineyval,
                                                A.directions,
                                                A.rail_type,
                                                A.nodetype,
												A.comments, 
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												A.Status,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
								FROM  robotsimgraph as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID  
										Where 		 A.simulayoutcd = simulayoutcds
												AND  A.startnode like CONCAT('%',startnodes, '%') 
											ORDER BY CAST( A.startnode AS DOUBLE) ,CAST( A.endnode AS DOUBLE);
									  END IF;
                                      
                                                                
									  
									  IF SPstatus='2' THEN 

									  IF EXISTS(SELECT 1 FROM robotsimgraph WHERE startnode = startnodes and endnode = endnodes and simulayoutcd=simulayoutcds  ) THEN 
                                      
												UPDATE robotsimgraph 
														SET  
														 distance = distances,
														 reachtime = reachtimes,
														 speed = speeds,
														 iconxval = iconxvals,
														 iconyval = iconyvals,
														 linexval = linexvals,
														 lineyval = lineyvals,
                                                         directions=directionss,
                                                         rail_type=rail_types,
														 nodetype=nodetypes,
														 comments  = commentss, 
														 IsApporved = IsApporveds,
													     Status  = IsApporveds, 
														 UpdatedBy  = UpdatedBys,
														 UpdatedDate  = CURRENT_TIMESTAMP
														WHERE  startnode = startnodes 
																AND endnode = endnodes 
                                                                AND simulayoutcd=simulayoutcds; 
												
											Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
							ELSE   
											INSERT INTO robotsimgraph
														( `simulayoutcd`,  
                                                        `startnode`,                                                       
														`endnode`,
														`distance`,
														`reachtime`,
														`speed`,
														`iconxval`,
														`iconyval`,
														`linexval`,
														`lineyval`,
                                                        `directions`,
                                                        `rail_type`,
														`nodetype`,
														`comments`, 
														`IsApporved`,
														`Status`,
														`CreatedBY`,
														`UpdatedBy`)
														VALUES
														(
														simulayoutcds,
                                                        startnodes,
														endnodes,
														distances,
														reachtimes,
														speeds,
														iconxvals,
														iconyvals,
														linexvals,
														lineyvals,
                                                        directionss,
                                                        rail_types,
														nodetypes,
														commentss, 
														IsApporveds,
														Statuss,
														UpdatedBys,
														UpdatedBys); 

											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF; 
										END IF;
                                        
				  IF SPstatus='4' THEN 
									Delete from robotsimgraph where  simulayoutcd = simulayoutcds ;        
                                	Select 'D101' as CODE,'okDeleted' as MSG ,'All datas deleted' as VALUE; 
                            END IF;        
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotSimulationTableGraphGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotSimulationTableGraphGetPost`(IN graphids INT,  
IN simulayoutcds  varchar(15),
IN tablearucos  varchar(45),
IN custtablenos  varchar(45),
IN endnodes varchar(45),
IN distances DECIMAL(10,2),
IN reachtimes DECIMAL(10,2),
IN speeds DECIMAL(10,2),
IN tasktypes varchar(45),
IN taskdetailtypes varchar(45),
IN iconxvals DECIMAL(10,2),
IN iconyvals DECIMAL(10,2),
IN linexvals DECIMAL(10,2),
IN lineyvals DECIMAL(10,2),
IN directionss varchar(45),
IN tableangles varchar(45),
IN commentss VARCHAR(100), 
IN priyorityuses varchar(10),
IN UpdatedBys varchar(10),
IN IsApporveds  varchar(10), 
IN column1s varchar(45),
IN column2s varchar(45),
IN column3s varchar(45),
IN column4s varchar(45),
IN column5s varchar(45),
IN SPstatus varchar(10)  )
BEGIN 

DECLARE  tasktypecd varchar(10);
DECLARE  taskdetailtypecd varchar(10);

					IF SPstatus='1' THEN 
									SELECT 	   '0' as rowsVal,
											    A.graphid,
                                                A.simulayoutcd,
												A.tablearuco,
                                                IFNULL(A.custtableno,"") as  custtableno,
												A.endnode,
                                                D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.distance,
												A.reachtime,
												A.speed,
												A.iconxval,
												A.iconyval,
												A.linexval,
												A.lineyval,
                                                A.directions,
                                                A.tableangle,
												A.comments, 
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												CASE   WHEN A.priyorityuse='1' THEN 'true' Else 'false' END as priyorityuse,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate,
                                                IFNULL(A.column1,"") as  column1,
												IFNULL(A.column2,"") as  column2,
												IFNULL(A.column3,"") as  column3,
												IFNULL(A.column4,"") as  column4,
												IFNULL(A.column5,"") as  column5 
								FROM  robotsimtablegraph as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID  
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD=A.tasktype
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD=A.taskdetailtype
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
										Where 		 A.simulayoutcd = simulayoutcds
												AND  A.tablearuco like CONCAT('%',tablearucos, '%') 
											ORDER BY  CAST( SUBSTRING(A.tablearuco, 2) AS DOUBLE) ,CAST( A.endnode AS DOUBLE);
									  END IF;
                                      
                                      
                                      IF SPstatus='3' THEN 
									SELECT 	   '0' as rowsVal,
											    A.graphid,
                                                A.simulayoutcd,
												A.tablearuco,
                                                IFNULL(A.custtableno,"") as  custtableno,
												A.endnode,
                                                D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.distance,
												A.reachtime,
												A.speed,
												A.iconxval,
												A.iconyval,
												A.linexval,
												A.lineyval,
                                                A.directions,
                                                A.tableangle,
												A.comments, 
												CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
												CASE   WHEN A.priyorityuse='1' THEN 'true' Else 'false' END as priyorityuse,
												A.CreatedBY,
												B.UserDispName as CreatedUser,
												DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
												A.UpdatedBy,
												C.UserDispName as UpdatedUser,
												DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate,
                                                IFNULL(A.column1,"") as  column1,
												IFNULL(A.column2,"") as  column2,
												IFNULL(A.column3,"") as  column3,
												IFNULL(A.column4,"") as  column4,
												IFNULL(A.column5,"") as  column5 
								FROM  robotsimtablegraph as A 
										Left outer Join usermaster as B ON
																A.CreatedBY = B.UserID
													 Left outer Join usermaster as C ON
																A.UpdatedBy = C.UserID  
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD=A.tasktype
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD=A.taskdetailtype
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
										Where 		 A.simulayoutcd = simulayoutcds
												AND  A.tablearuco = tablearucos
											ORDER BY  CAST( SUBSTRING(A.tablearuco, 2) AS DOUBLE)  ,CAST( A.endnode AS DOUBLE);
									  END IF;
									  
									  IF SPstatus='2' THEN 
  SET tasktypecd = (Select MINOR_CD FROM commoncode Where CD_NM=tasktypes AND  MINOR_CD !='*****' AND  Major_CD ='A007'	AND   IsApporved='1' ); 
  SET taskdetailtypecd = (Select MINOR_CD FROM commoncode Where CD_NM=taskdetailtypes AND  MINOR_CD !='*****' AND  Major_CD ='A008'	AND   IsApporved='1' ); 
 
									  IF EXISTS(SELECT 1 FROM robotsimtablegraph WHERE tablearuco = tablearucos and endnode = endnodes and simulayoutcd=simulayoutcds  ) THEN 
                                      
												UPDATE robotsimtablegraph 
														SET  
                                                        custtableno = custtablenos,
														 distance = distances,
														 reachtime = reachtimes,
														 speed = speeds,
                                                         tasktype=tasktypecd,
                                                         taskdetailtype = taskdetailtypecd,
														 iconxval = iconxvals,
														 iconyval = iconyvals,
														 linexval = linexvals,
														 lineyval = lineyvals,
                                                         directions=directionss,
                                                         tableangle=tableangles,
														 comments  = commentss, 
                                                         column1	= column1s,
														 column2	= column2s,
														 column3	= column3s,
														 column4	= column4s,
														 column5	= column5s,
														 IsApporved = IsApporveds,
													     priyorityuse  = priyorityuses, 
														 UpdatedBy  = UpdatedBys,
														 UpdatedDate  = CURRENT_TIMESTAMP
														WHERE tablearuco = tablearucos 
                                                        AND endnode = endnodes 
                                                        AND simulayoutcd=simulayoutcds; 
												
											Select 'U101' as CODE,'okSaved' as MSG ,'AMR Names data updated' as VALUE; 
							ELSE   
											INSERT INTO robotsimtablegraph
														( `simulayoutcd`,  
                                                        `tablearuco`,   
                                                        `custtableno`,
														`endnode`,
														`distance`,
														`reachtime`,
														`speed`,
                                                        `tasktype`,
                                                        `taskdetailtype`,
														`iconxval`,
														`iconyval`,
														`linexval`,
														`lineyval`,
                                                        `directions`,
                                                        `tableangle`,
														`comments`, 
														`IsApporved`,
														`priyorityuse`,
														`CreatedBY`,
														`UpdatedBy`,
                                                        `column1`,
													    `column2`,
													    `column3`,
													    `column4`,
													    `column5`)
														VALUES
														(
														simulayoutcds,
                                                        tablearucos,
                                                        custtablenos,
														endnodes,
														distances,
														reachtimes,
														speeds,
                                                        tasktypecd,
                                                        taskdetailtypecd,
														iconxvals,
														iconyvals,
														linexvals,
														lineyvals,
                                                        directionss,
                                                        tableangles,
														commentss, 
														IsApporveds,
														priyorityuses,
														UpdatedBys,
														UpdatedBys,
                                                        column1s,
														column2s,
														column3s,
														column4s,
														column5s); 

											Select 'I101' as CODE,'okSaved' as MSG ,'Data inserted' as VALUE; 
							END IF; 
										END IF;
                                        
                           IF SPstatus='4' THEN 
								 Delete from robotsimtablegraph where simulayoutcd = simulayoutcds ;        
                                	Select 'D101' as CODE,'okDeleted' as MSG ,'All datas deleted' as VALUE; 
                            END IF;        
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotSimuOrderExecuteMDGETPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotSimuOrderExecuteMDGETPost`(   
IN orderexecmids VARCHAR(20),
IN startDates VARCHAR(20),
IN endDates VARCHAR(20), 
IN worknames  VARCHAR(100),
IN robotips  VARCHAR(30),
IN commentss  VARCHAR(100),
IN ordertypes  VARCHAR(20),
IN orderstatuss  VARCHAR(20),
IN startworkstations  VARCHAR(20),
IN finalworkstations  VARCHAR(20),
IN completestatuss  varchar(10),
IN workstatuss  varchar(10),
IN dtlcomments  varchar(200),
IN UpdatedBys VARCHAR(20),
IN SPstatus varchar(10)  
)
BEGIN	  
DECLARE  orderexecmidM varchar(10);

				IF SPstatus='1' THEN
								SELECT 		  CAST(A.orderexecmid  as CHAR(50))   as rowsVal,  
												A.robotip,
												A.workname,	
												B.startworkstation,
												B.finalworkstation,
												D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.ordertype,
												B.completestatus ,
												A.orderstatus as workstatus,
                                                A.comments,
                                                CONVERT(A.createdDate,char) as createdDate,
												F.UserDispName as updatedBy,
												CONVERT(B.updatedDate,char) as  updatedDate
								 FROM  `robotsimorderexecmaster`  as A
														Left outer Join  robotsimorderexecdetail as B ON
																			A.orderexecmid = B.orderexecmid
														Left outer Join  robotmaster as C ON
																			A.robotip = C.robotmipaddress
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD='O3'
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD='OD1'
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
															Left Outer join usermaster as F On
																A.UpdatedBy = F.UserId     
								Where 	 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') =  DATE_FORMAT(CURRENT_TIMESTAMP, '%Y-%m-%d')
										Order by   convert(B.updatedDate , datetime)  DESC , Cast(B.orderexecdid as double) Desc, Cast(B.startworkstation as double) Desc, Cast(B.finalworkstation as double) ;
					END IF;
                    
                    
                    IF SPstatus='2' THEN
								SELECT 		  CAST(A.orderexecmid  as CHAR(50))   as rowsVal,  
												A.robotip,
												A.workname,	
												B.startworkstation,
												B.finalworkstation,
												D.CD_NM as tasktype,
												E.CD_NM as taskdetailtype,
												A.ordertype,
												B.completestatus ,
												A.orderstatus as workstatus,
                                                A.comments,
                                                CONVERT(A.createdDate,char) as createdDate,
												F.UserDispName as updatedBy,
												CONVERT(B.updatedDate,char) as  updatedDate
								 FROM  `robotsimorderexecmaster`  as A
														Left outer Join  robotsimorderexecdetail as B ON
																			A.orderexecmid = B.orderexecmid
														Left outer Join  robotmaster as C ON
																			A.robotip = C.robotmipaddress
														Left outer Join  commoncode as D ON
																			 D.MINOR_CD='O3'
																	AND  D.MINOR_CD !='*****'
																	AND  D.Major_CD ='A007'
																	AND   D.IsApporved='1'  
														Left outer Join  commoncode as E ON
																			 E.MINOR_CD='OD1'
																	AND  E.MINOR_CD !='*****'
																	AND  E.Major_CD ='A008'
																	AND   E.IsApporved='1'  
															Left Outer join usermaster as F On
																A.UpdatedBy = F.UserId     
									Where 			 A.workname like CONCAT('%', worknames, '%')   
											AND		 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') BETWEEN startDates AND endDates
										Order by   convert(B.updatedDate , datetime)  DESC , Cast(B.orderexecdid as double) Desc, Cast(B.startworkstation as double) Desc, Cast(B.finalworkstation as double) ;
				
					END IF;
                    
                    
          IF SPstatus='3' THEN 
                  IF EXISTS(SELECT 1 FROM robotsimorderexecmaster WHERE orderexecmid = orderexecmids ) THEN
					set orderexecmidM =orderexecmids;
                                
                    UPDATE  `robotsimorderexecmaster`
								SET  
                                `orderstatus`=orderstatuss, 
                                 `comments`=commentss, 
								`UpdatedBy` = UpdatedBys,
								`UpdatedDate` = CURRENT_TIMESTAMP
								WHERE orderexecmid = orderexecmids;
                                
                                   INSERT INTO `robotsimorderexecdetail`
											(`orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecmidM,  
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);
                                
                          
                    
						Select 'U101' as CODE,'okSaved' as MSG ,taskexecMIDsM as VALUE;

        ELSE        
             SET orderexecmidM = (Select Case WHEN (Select  max(orderexecmid)+1 from `robotsimorderexecmaster`) IS NULL  THEN 1 ELSE
(Select  max(orderexecmid)+1 from `robotsimorderexecmaster`) END); 

						INSERT INTO `robotsimorderexecmaster`
										(`orderexecmid`,
										`workname`,
										`robotip`,
										`ordertype`,
										`orderstatus`,
										`comments`,
										`CreatedBY`, 
										`UpdatedBy`)
										VALUES
										(orderexecmidM,
										worknames,
										robotips,
										ordertypes,
										orderstatuss,
										commentss,
										UpdatedBys,
										UpdatedBys);
                                        
                          INSERT INTO `robotsimorderexecdetail`
											(`orderexecmid`, 
											`startworkstation`,
                                            `finalworkstation`,
                                            `completestatus`,
                                            `workstatus`,
                                            `comments`,
											`CreatedBY`, 
											`UpdatedBy`)
											VALUES
											(orderexecmidM,  
											startworkstations,
                                            finalworkstations,
                                            completestatuss,
                                            workstatuss,
                                            dtlcomments,
											UpdatedBys,
											UpdatedBys);


						Select 'I101' as CODE,'okSaved' as MSG ,orderexecmidM as VALUE;

        END IF;
                  
                    END IF;          
           
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotTaskChainMDGETPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotTaskChainMDGETPost`(IN  taskids  varchar(10),
IN orderexecmids VARCHAR(20), 
IN robotips varchar(20),
IN worknames varchar(10),
IN simulayoutcdss varchar(10),
IN startworkstations varchar(10),
IN endworkstations varchar(10),
IN trayracks varchar(10),
IN taskrunoks varchar(10),
IN tasktypes varchar(10),
IN orderstatuss varchar(10),
IN prioritys varchar(10),
IN emergencys varchar(10),
IN sortorders varchar(10),
IN commentss varchar(10),
IN UpdatedBys varchar(10),
IN column1s varchar(45),
IN column2s varchar(45),
IN column3s varchar(45),
IN column4s varchar(45),
IN column5s varchar(45),
IN IsApporveds  varchar(10),
IN SPstatus varchar(10))
BEGIN

	DECLARE  maxSortno varchar(10);
							IF SPstatus='9' THEN 
								SELECT  '0' as rowsVal,
										 A.taskid ,
										 A.robotip,
										 A.workname,
                                         A.simulayoutcds,
										 IFNULL(A.startworkstation,"") startworkstation,
										 IFNULL(A.endworkstation,"") as endworkstation,
										 IFNULL(A.trayrack,"") as trayrack, 
										 IFNULL(A.taskrunok,"0") as  taskrunok,
										 IFNULL(A.tasktype,"2") as tasktype,
                                         IFNULL(A.tasktype,"2") as tasknm,
                                         '' as  trayracknm,
                                       /* IFNULL(B.CD_NM,"") as  trayracknm, 
										CASE
											WHEN A.tasktype = 2 THEN "서빙"
											WHEN A.tasktype = 3 THEN "현금"
                                            WHEN A.tasktype = 4 THEN "회수"
											ELSE "서빙"  END as tasknm,
										 CASE
											WHEN A.orderstatus = 1 THEN "대기"
											WHEN A.orderstatus = 2 THEN "출발"
                                            WHEN A.orderstatus = 3 THEN "주행"
                                            WHEN A.orderstatus = 4 THEN "정지"
                                            WHEN A.orderstatus = 5 THEN "실패"
                                            WHEN A.orderstatus = 6 THEN "주행완료"
                                            WHEN A.orderstatus = 7 THEN "취소"
											ELSE "대기" END as Orderstatusnm ,*/

										 IFNULL(A.orderstatus,"1") as  Orderstatusnm,
                                         IFNULL(A.orderstatus,"2") as orderstatus,
										 IFNULL(A.priority,"0") as  priority,
										 IFNULL(A.emergency,"0") as  emergency,
										 IFNULL(A.sortorder,"0") as  sortorder,
										 IFNULL(A.comments,"") as comments
									 /* ,IFNULL(C.UserDispName,"")   as  createdby,
										 CONVERT(A.createdDate,char) as  createddate,
										 IFNULL(D.UserDispName,"")   as  updatedBy,
										 CONVERT(A.UpdatedDate,char) as  updateddate,
										 IFNULL(A.column1,"") as  column1,
										 IFNULL(A.column2,"") as  column2,
										 IFNULL(A.column3,"") as  column3,
										 IFNULL(A.column4,"") as  column4,
										 IFNULL(A.column5,"") as  column5 */
									FROM  taskchain as A 
												Left outer Join  commoncode as B ON
															   B.MINOR_CD  = A.trayrack
														  AND  B.MINOR_CD  != '*****'
														  AND  B.Major_CD   = 'A004'
														  AND  B.IsApporved  = '1'
                                                Left Outer join usermaster as C On
																A.CreatedBY = C.UserId  
												Left Outer join usermaster as D On
																A.UpdatedBy = D.UserId  
									 WHERE  	A.tasktype <= 4
										   AND  A.orderstatus >=2 AND A.orderstatus <= 5
									ORDER BY   A.sortorder , A.tasktype ,A.createdDate ;
                END IF;
                
				IF SPstatus='1' THEN 
								SELECT  '0' as rowsVal,
										 A.taskid ,
										 A.robotip,
										 A.workname,
                                         A.simulayoutcds,
										 IFNULL(A.startworkstation,"") startworkstation,
										 IFNULL(A.endworkstation,"") as endworkstation,
										 IFNULL(A.trayrack,"") as trayrack, 
										 IFNULL(A.taskrunok,"0") as  taskrunok,
										 IFNULL(A.tasktype,"2") as tasktype,
                                         IFNULL(A.tasktype,"2") as tasknm,
                                         '' as  trayracknm,
                                       /* IFNULL(B.CD_NM,"") as  trayracknm, 
										CASE
											WHEN A.tasktype = 2 THEN "서빙"
											WHEN A.tasktype = 3 THEN "현금"
                                            WHEN A.tasktype = 4 THEN "회수"
											ELSE "서빙"  END as tasknm,
										 CASE
											WHEN A.orderstatus = 1 THEN "대기"
											WHEN A.orderstatus = 2 THEN "출발"
                                            WHEN A.orderstatus = 3 THEN "주행"
                                            WHEN A.orderstatus = 4 THEN "정지"
                                            WHEN A.orderstatus = 5 THEN "실패"
                                            WHEN A.orderstatus = 6 THEN "주행완료"
                                            WHEN A.orderstatus = 7 THEN "취소"
											ELSE "대기" END as Orderstatusnm ,*/

										 IFNULL(A.orderstatus,"1") as  Orderstatusnm,
                                         IFNULL(A.orderstatus,"2") as orderstatus,
										 IFNULL(A.priority,"0") as  priority,
										 IFNULL(A.emergency,"0") as  emergency,
										 IFNULL(A.sortorder,"0") as  sortorder,
										 IFNULL(A.comments,"") as comments
									 /* ,IFNULL(C.UserDispName,"")   as  createdby,
										 CONVERT(A.createdDate,char) as  createddate,
										 IFNULL(D.UserDispName,"")   as  updatedBy,
										 CONVERT(A.UpdatedDate,char) as  updateddate,
										 IFNULL(A.column1,"") as  column1,
										 IFNULL(A.column2,"") as  column2,
										 IFNULL(A.column3,"") as  column3,
										 IFNULL(A.column4,"") as  column4,
										 IFNULL(A.column5,"") as  column5 */
									FROM  taskchain as A 
												Left outer Join  commoncode as B ON
															   B.MINOR_CD  = A.trayrack
														  AND  B.MINOR_CD  != '*****'
														  AND  B.Major_CD   = 'A004'
														  AND  B.IsApporved  = '1'
                                                Left Outer join usermaster as C On
																A.CreatedBY = C.UserId  
												Left Outer join usermaster as D On
																A.UpdatedBy = D.UserId  
									 WHERE  	A.tasktype <= 4
										   AND  A.orderstatus <=1
									ORDER BY   A.sortorder , A.tasktype ,A.createdDate ;
                END IF;
                
                IF SPstatus='2' THEN 
								SELECT    '0' as rowsVal,
										   A.taskid ,
										   A.robotip,
										   CASE WHEN LEFT(A.workname , 1) ="H" THEN  "홈"
                                           ELSE CONCAT( "테이블 #",  SUBSTR(A.workname,2)) END as workname,
                                           A.simulayoutcds,
										   IFNULL(A.startworkstation,"") startworkstation,
										   IFNULL(A.endworkstation,"") as endworkstation,
										   IFNULL(A.trayrack,"") as trayrack, 
										   IFNULL(A.taskrunok,"0") as  taskrunok,
										   IFNULL(A.tasktype,"2") as tasktype,
                                           IFNULL(A.tasktype,"2") as tasknm, 
                                           IFNULL(B.CD_NM,"") as  trayracknm, 
									    /* CASE
											WHEN A.tasktype = 2 THEN "서빙"
											WHEN A.tasktype = 3 THEN "현금"
                                            WHEN A.tasktype = 4 THEN "회수"
											ELSE "서빙"  END as tasknm,
										 CASE
											WHEN A.orderstatus = 1 THEN "대기"
											WHEN A.orderstatus = 2 THEN "출발"
                                            WHEN A.orderstatus = 3 THEN "주행"
                                            WHEN A.orderstatus = 4 THEN "정지"
                                            WHEN A.orderstatus = 5 THEN "실패"
                                            WHEN A.orderstatus = 6 THEN "주행완료"
                                            WHEN A.orderstatus = 7 THEN "취소"
											ELSE "대기" END as Orderstatusnm ,*/
 
                                         IFNULL(A.orderstatus,"2") as orderstatus,
										 IFNULL(A.priority,"0") as  priority,
										 IFNULL(A.emergency,"0") as  emergency,
										 IFNULL(A.sortorder,"0") as  sortorder,
										 IFNULL(A.comments,"") as comments,
									 /* ,IFNULL(C.UserDispName,"")   as  createdby,
										 CONVERT(A.createdDate,char) as  createddate,
										 IFNULL(D.UserDispName,"")   as  updatedBy,
										 CONVERT(A.UpdatedDate,char) as  updateddate,
										 IFNULL(A.column1,"") as  column1,
										 IFNULL(A.column2,"") as  column2,
										 IFNULL(A.column3,"") as  column3,
										 IFNULL(A.column4,"") as  column4,
										 IFNULL(A.column5,"") as  column5 */
                                         
										 IFNULL(D.UserDispName,"")   as  updatedBy, 
										  IFNULL(CONVERT(E.updatedDate,char),A.updatedDate) as  updatedDate,
                                         replace(IFNULL(SUBSTRING(TIMEDIFF(IFNULL(CONVERT(E.createdDate,char),A.createdDate),
										 IFNULL(CONVERT(E.updatedDate,char),A.createdDate) ),5,5),''),".","") as totaltimes
									FROM  taskchain as A 
												Left outer Join  commoncode as B ON
															   B.MINOR_CD  = A.trayrack
														  AND  B.MINOR_CD  != '*****'
														  AND  B.Major_CD   = 'A004'
														  AND  B.IsApporved  = '1'
                                                Left Outer join usermaster as C On
																A.CreatedBY = C.UserId  
												Left Outer join usermaster as D On
																A.UpdatedBy = D.UserId  
												Left Outer Join robotorderexecmaster as E ON
																A.taskid = E.taskid
									 WHERE 		 A.workname like CONCAT('%', worknames, '%')  
											AND	 DATE_FORMAT(A.updatedDate, '%Y-%m-%d') BETWEEN column1s AND column2s 
									ORDER BY  IFNULL(CONVERT(E.updatedDate,char),A.createdDate) desc;
                END IF;
                
                   IF SPstatus='3' THEN 
                   
                      IF EXISTS(SELECT 1 FROM taskchain WHERE taskid = taskids) THEN
				                 
									UPDATE  taskchain 
											SET 
											`robotip` = robotips ,
											`workname` = worknames ,
											`startworkstation` =  startworkstations ,
											`endworkstation` =  endworkstations ,
											`trayrack` =  trayracks ,
											`taskrunok` =  taskrunoks ,
											`tasktype` =  tasktypes ,
											`orderstatus` =  orderstatuss ,
											`priority` =  prioritys ,
											`emergency` =  emergencys ,
											`sortorder` =  sortorders ,
											`comments` =  commentss ,
											`UpdatedBy` =  UpdatedBys , 
											`UpdatedDate` =  CURRENT_TIMESTAMP ,
											`column1` =  column1s ,
											`column2` =  column2s ,
											`column3` =  column3s ,
											`column4` =  column4s ,
											`column5` =  column5s  
									 WHERE taskid = taskids;
                        
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;

					ELSE  
                  /*    IF EXISTS(SELECT 1 FROM taskchain WHERE robotip = robotips AND workname = worknames AND tasktype=tasktypes  AND orderstatus <=5 ) THEN
                   */
                   IF EXISTS(SELECT 1 FROM taskchain WHERE robotip = robotips AND workname = worknames   AND orderstatus <=5 ) THEN
                 
                   Select 'E101' as CODE,'DataExist' as MSG ,orderexecmids as VALUE;
                     ELSE 
                     
                       SET maxSortno = (  SELECT IFNULL(MAX(sortorder)+1,"1")  FROM taskchain WHERE orderstatus <=5 and tasktype >1  ); 
                       
                       
                       if worknames="H1" then
                        SET maxSortno = "0";
                        SET taskrunoks="1";
                       end if;
  
							INSERT INTO  taskchain 
												( taskid,
												  robotip,
												  workname,
                                                  simulayoutcds,
												  startworkstation,
												  endworkstation,
												  trayrack,
												  taskrunok,
												  tasktype,
												  orderstatus,
												  priority,
												  emergency,
												  sortorder,
												  comments,
												  CreatedBY, 
												  UpdatedBy, 
												  column1,
												  column2,
												  column3,
												  column4,
												  column5)
												VALUES
												( taskids,
												  robotips,
												  worknames,
                                                  simulayoutcdss,
												  startworkstations,
												  endworkstations,
												  trayracks,
												  taskrunoks,
												  tasktypes,
												  orderstatuss,
												  prioritys,
												  emergencys,
												  maxSortno,
												  commentss,
												  UpdatedBys, 
												  UpdatedBys, 
												  column1s,
												  column2s,
												  column3s,
												  column4s,
												  column5s);


							Select 'I101' as CODE,'okSaved' as MSG ,'Inserted' as VALUE;
							END IF;
						END IF;
                  
                    END IF;      
                    
                       IF SPstatus='4' THEN  
									UPDATE  taskchain 
											SET  
											`orderstatus` =  orderstatuss , 
											`comments` =  commentss ,
											`UpdatedBy` =  UpdatedBys , 
											`UpdatedDate` =  CURRENT_TIMESTAMP   
									 WHERE taskid = taskids; 
                                     
                                     
                                       UPDATE  `robotorderexecmaster`
												SET   
												`orderstatus`=orderstatuss, 
												 `comments`=commentss, 
												`UpdatedBy` = UpdatedBys,
												`UpdatedDate` = CURRENT_TIMESTAMP
												WHERE  taskid = taskids; 
                                                
                                   UPDATE  currtasksetting 
									  SET  	   `workstatus`= workstatuss,
                                               `orderstatus` = orderstatuss 
                                               WHERE  taskid = taskids; 
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE; 
                  
                    END IF;
                    
                      IF SPstatus='5' THEN  
									UPDATE  taskchain 
											SET   
											`workname` = worknames , 
											`trayrack` =  trayracks , 
											`tasktype` =  tasktypes ,
											`orderstatus` =  orderstatuss , 
											`comments` =  commentss ,
											`UpdatedBy` =  UpdatedBys 
									 WHERE taskid = taskids; 
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE;  
                    END IF; 
                    
                     IF SPstatus='6' THEN  
									UPDATE  taskchain 
											SET  
											`trayrack` =  trayracks , 
											`tasktype` =  tasktypes ,
											`orderstatus` =  orderstatuss , 
											`comments` =  commentss ,
											`UpdatedBy` =  UpdatedBys ,
											`UpdatedDate` =  CURRENT_TIMESTAMP   
									 WHERE taskid = taskids;  
                                     
                                       UPDATE  `robotorderexecmaster`
												SET `tasktype` =  tasktypes ,  
												`orderstatus`=orderstatuss, 
												 `comments`=commentss, 
												`UpdatedBy` = UpdatedBys,
												`UpdatedDate` = CURRENT_TIMESTAMP
												WHERE  taskid = taskids; 
                                                
									   UPDATE  currtasksetting 
										  SET  	   `tasktype` =  tasktypes ,
													`workstatus`= workstatuss,
													`orderstatus` = orderstatuss 
												   WHERE  taskid = taskids; 
                    
						Select 'U101' as CODE,'okSaved' as MSG ,orderexecmids as VALUE; 
                  
                    END IF; 
                    
                     /*   Sort the Task
                   */
                    IF SPstatus='7' THEN  
									UPDATE  taskchain 
											SET  
											`sortorder` =  sortorders ,  
											`UpdatedDate` =  CURRENT_TIMESTAMP   
									 WHERE taskid = taskids;   
                    
						Select 'U106' as CODE,'okUpdated' as MSG ,taskids as VALUE; 
                  
                    END IF;
                    
                    IF SPstatus='8' THEN  
									UPDATE  taskchain 
											SET  
											`sortorder` =  sortorders ,  
											`UpdatedDate` =  CURRENT_TIMESTAMP   
									 WHERE taskid = taskids;   
                    
						Select 'U106' as CODE,'okUpdated' as MSG ,taskids as VALUE; 
                  
                    END IF;
                    
                    
                     IF SPstatus='10' THEN  									
									Update taskchain 
											SET  
											`orderstatus` =  7 ,  
											`UpdatedDate` =  CURRENT_TIMESTAMP   
									where orderstatus >1 && orderstatus <6;   
						Select 'U101' as CODE,'ok Task Canceled' as MSG ,robotips as VALUE; 
                    END IF; 
                     
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RobotTexttoSpeechPlay` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'ONLY_FULL_GROUP_BY,STRICT_TRANS_TABLES,NO_ZERO_IN_DATE,NO_ZERO_DATE,ERROR_FOR_DIVISION_BY_ZERO,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RobotTexttoSpeechPlay`(  
IN  texttoplays varchar(45),
IN  langtypes varchar(45),
IN  noofplayss varchar(45),
IN  playstatuss varchar(45),
IN  col1s varchar(45),
IN  col2s varchar(45), 
IN  SPstatus varchar(10)  )
BEGIN 

					IF SPstatus='1' THEN 
								SELECT		texttoplay,
											langtype,
											noofplays,
											playstatus,
											col1,
											col2
								FROM  texttospeechplay;
				     END IF;
                                      
					 IF SPstatus='2' THEN 
							    
                        IF EXISTS(SELECT 1 FROM texttospeechplay) THEN
				                 
								UPDATE  `texttospeechplay`
										SET
										`texttoplay`  = texttoplays,
										`langtype`    = langtypes,
										`noofplays`   = noofplayss,
										`playstatus`  = playstatuss,
                                         col1		  = col1s,
										 col2		  = col2s ; 
								Select 'U101' as CODE,'okSaved' as MSG ,texttoplays as VALUE; 
					ELSE    
								INSERT INTO  `texttospeechplay`
											(`texttoplay`,
											`langtype`,
											`noofplays`,
											`playstatus`,
											`col1`,
											`col2`)
											VALUES
											( texttoplays,
											  langtypes,
											  noofplayss,
											  playstatuss,
											  col1s,
											  col2s); 

							Select 'I101' as CODE,'okSaved' as MSG ,'Inserted' as VALUE;
						 
						END IF;
               END IF;
               
               
                IF SPstatus='3' THEN  
								UPDATE  `texttospeechplay`
										SET 
										`playstatus`  = playstatuss  ; 
								Select 'U101' as CODE,'okSaved' as MSG ,texttoplays as VALUE; 
					 
               END IF;
               
                IF SPstatus='4' THEN  
								UPDATE  `texttospeechplay`
										SET 
										`col1`  = col1s  ;  
								Select 'U101' as CODE,'okSaved' as MSG ,texttoplays as VALUE; 
					 
               END IF;
	
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_RolesGetPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_RolesGetPost`(IN RoleIDs INT, 
IN RoleNames varchar(50),
IN Commentss varchar(100),
IN UpdatedBys INT,
IN SPstatus varchar(10)
)
BEGIN
		IF SPstatus='1' THEN
					SELECT '0' as RowsVal,
								A.RoleID,
								A.RoleName,
								A.Comments,
								A.CreatedBY,
                                B.UserDispName as CreatedUser,
								 DATE_FORMAT(A.CreatedDate, '%Y-%m-%d') AS CreatedDate,
								A.UpdatedBy,
                                C.UserDispName as UpdatedUser,
								 DATE_FORMAT(A.UpdatedDate, '%Y-%m-%d') AS UpdatedDate
					FROM `rolemaster` as A
							 Left outer Join usermaster as B ON
											A.CreatedBY = B.UserID
								 Left outer Join usermaster as C ON
											A.UpdatedBy = C.UserID 
							Where  A.RoleName like CONCAT('%', RoleNames, '%'); 
        END IF;  
        
        IF SPstatus='2' THEN
        
        IF EXISTS(SELECT 1 FROM `rolemaster` WHERE RoleID = RoleIDs) THEN
					UPDATE `rolemaster`
							SET
							 `RoleName` = RoleNames,
							`Comments` = Commentss, 
							`UpdatedBy` = UpdatedBys
							WHERE `RoleID` = RoleIDs;
                            
						Select 'U101' as CODE,'okSaved' as MSG ,'Roles data updated' as VALUE;

        ELSE
						INSERT INTO `rolemaster`
									(`RoleName`,
									`Comments`,
									`CreatedBY`, 
									`UpdatedBy`)
									VALUES
									(RoleNames,
									Commentss,
									UpdatedBys,
									UpdatedBys);
						Select 'I101' as CODE,'okSaved' as MSG ,'Roles data inserted' as VALUE;

        END IF;
        
        END IF;  
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_userGet` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_userGet`(IN UserIDs varchar(50), 
IN UserNames varchar(50),
IN emails varchar(100),
IN SPstatus varchar(10) 
)
BEGIN
					IF SPstatus='1' THEN
					SELECT 		A.UserId,
								A.UserName,
								A.RoleID,
                                B.RoleName,
								A.Password,
								A.EMPNO,
								A.UserDispName,
								A.Email,
								A.PHNO,
								A.Address,
								CASE   WHEN A.IsApporved='1' THEN 'true' Else 'false' END as IsApporved,
								CASE   WHEN A.Status='1' THEN 'true' Else 'false' END as Status
						FROM `usermaster` as A  
                          Left Outer Join `rolemaster` as B  
									ON A.RoleID = B.RoleID
							Where  A.UserName like CONCAT('%', UserNames, '%')
                            AND A.UserDispName like CONCAT('%', emails, '%'); 
                          
        END IF;  
        
        IF SPstatus='2' THEN
				UPDATE `usermaster`
							SET 
							`IsApporved` = 0,
							`Status` = 0
							WHERE `UserId` = UserIDs ;
			Select 'U101' as CODE,'okDeleted' as MSG ,'Roles data updated' as VALUE; 
        END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_userlogin` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_userlogin`(IN UserNames varchar(50),  
IN Passwords varchar(50)   )
BEGIN
		SELECT userid,empno,userName,UserDispName,password,RoleID,Email,PHNO,Address FROM usermaster
		where
		userName LIKE CONCAT('%', UserNames , '%') 
		and
		password LIKE  CONCAT('%', Passwords , '%') ;


END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!50003 DROP PROCEDURE IF EXISTS `sp_userPost` */;
/*!50003 SET @saved_cs_client      = @@character_set_client */ ;
/*!50003 SET @saved_cs_results     = @@character_set_results */ ;
/*!50003 SET @saved_col_connection = @@collation_connection */ ;
/*!50003 SET character_set_client  = utf8mb4 */ ;
/*!50003 SET character_set_results = utf8mb4 */ ;
/*!50003 SET collation_connection  = utf8mb4_0900_ai_ci */ ;
/*!50003 SET @saved_sql_mode       = @@sql_mode */ ;
/*!50003 SET sql_mode              = 'STRICT_TRANS_TABLES,NO_ENGINE_SUBSTITUTION' */ ;
DELIMITER ;;
CREATE DEFINER=`tact`@`%` PROCEDURE `sp_userPost`(IN UserIDs varchar(50), 
IN UserNames varchar(50),
IN Passwords varchar(100),
IN EMPNOs varchar(50),
IN UserDispNames varchar(50),
IN RoleIDs INT,
IN emailss varchar(60),
IN PHNOs varchar(20),
IN Addresss varchar(100),
IN IsApporveds INT,
IN Statuss INT,
IN SPstatus varchar(10)  
)
BEGIN
			IF SPstatus='1' THEN
						INSERT INTO `usermaster`
										( `UserName`,
										`RoleID`,
										`Password`,
										`EMPNO`,
										`UserDispName`,
										`Email`,
										`PHNO`,
										`Address`,
										`IsApporved`,
										`Status`)
										VALUES
										( UserNames,
                                        RoleIDs,
										 Passwords,
										 EMPNOs,
										 UserDispNames,
										 emailss,
										 PHNOs,
										 Addresss,
										 IsApporveds,
										 Statuss );
					Select 'U101' as CODE,'okSaved' as MSG ,'User data Inserted' as VALUE; 

            END IF;
            
            
            	IF SPstatus='2' THEN
								UPDATE `usermaster`
									SET
								    `UserName` = UserNames,
									`RoleID` = RoleIDs,
									`Password` = Passwords,
									`EMPNO` = EMPNOs,
									`UserDispName` =UserDispNames,
									`Email` =emailss,
									`PHNO` = PHNOs,
									`Address` =Addresss,
									`IsApporved` = IsApporveds,
									`Status` = Statuss
									WHERE `UserId` = UserIDs;
						Select 'U101' as CODE,'okSaved' as MSG ,'User data updated' as VALUE; 
				END IF;
END ;;
DELIMITER ;
/*!50003 SET sql_mode              = @saved_sql_mode */ ;
/*!50003 SET character_set_client  = @saved_cs_client */ ;
/*!50003 SET character_set_results = @saved_cs_results */ ;
/*!50003 SET collation_connection  = @saved_col_connection */ ;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

