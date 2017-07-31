#include "stdafx.h"
#include <mysql.h>
#include <stdio.h>
#include <stdlib.h>
#include "SQLConnect.h"

struct connection_details {
	char *server;
	char *user;
	char *password;
	char *database;
};

SQLConnect::SQLConnect() {
}

MYSQL *SQLConnect::setUpConnection(char* server, char* username, char* password, char* db) {
	MYSQL *conn;		// the connection
	struct connection_details mysqlD;
	mysqlD.server = server;  // where the mysql database is
	mysqlD.user = username;		// the root user of mysql	
	mysqlD.password = password; // the password of the root user in mysql
	mysqlD.database = db;	// the databse to pick

							// connect to the mysql database
	return conn = mysql_connection_setup(mysqlD);
}

int SQLConnect::getSelectedModel(MYSQL * connection) {
	modelSelected = 0;
	MYSQL_RES *res;	// the results
	MYSQL_ROW row;	// the results row (line by line)

	res = mysql_perform_query(connection, "SELECT id FROM argo_app_steps WHERE modelSelected = 1");
	while ((row = mysql_fetch_row(res)) != NULL)
		modelSelected += atoi(row[0]);

	return modelSelected;
}

int SQLConnect::getCurrentStep(MYSQL * connection) {
	MYSQL_RES *res;	// the results
	MYSQL_ROW row;	// the results row (line by line)

	std::ostringstream buf;
	buf << "SELECT currentStep FROM argo_app_steps WHERE id = '" << modelSelected << "' ";
	string str = buf.str();

	char* s2 = (char *)alloca(str.size() + 1);
	memcpy(s2, str.c_str(), str.size() + 1);
	res = mysql_perform_query(connection, s2);
	while ((row = mysql_fetch_row(res)) != NULL)
		currentStep += atoi(row[0]);

	return currentStep;
}

int SQLConnect::getMaxStep(MYSQL * connection) {
	MYSQL_RES *res;	// the results
	MYSQL_ROW row;	// the results row (line by line)

	std::ostringstream buf;
	buf << "SELECT maxSteps FROM argo_app_steps WHERE id = '" << modelSelected << "' ";
	string str = buf.str();

	char* s2 = (char *)alloca(str.size() + 1);
	memcpy(s2, str.c_str(), str.size() + 1);
	res = mysql_perform_query(connection, s2);
	while ((row = mysql_fetch_row(res)) != NULL)
		maxStep += atoi(row[0]);

	return maxStep;
}

void SQLConnect::updateNextStep(MYSQL * connection, int nextStep) {
	//UPDATE `argo`.`argo_app_steps` SET `currentStep`='5' WHERE `id`='1';
	MYSQL_RES *res;	// the results
//	MYSQL_ROW row;	// the results row (line by line)

	std::ostringstream buf;
	buf << "UPDATE `argo`.`argo_app_steps` SET `currentStep`= '" << nextStep << "'" << " WHERE id = '" << modelSelected << "'";

	string str = buf.str();

	char* s2 = (char *)alloca(str.size() + 1);
	memcpy(s2, str.c_str(), str.size() + 1);
	res = mysql_perform_query(connection, s2);

}


// just going to input the general details and not the port numbers


MYSQL* SQLConnect::mysql_connection_setup(struct connection_details mysql_details) {
	// first of all create a mysql instance and initialize the variables within
	MYSQL *connection = mysql_init(NULL);

	// connect to the database with the details attached.
	if (!mysql_real_connect(connection, mysql_details.server, mysql_details.user, mysql_details.password, mysql_details.database, 0, NULL, 0)) {
		printf("Conection error : %s\n", mysql_error(connection));
		exit(1);
	}
	return connection;
}

MYSQL_RES* SQLConnect::mysql_perform_query(MYSQL *connection, char *sql_query) {
	// send the query to the database
	if (mysql_query(connection, sql_query)) {
		printf("MySQL query error : %s\n", mysql_error(connection));
		exit(1);
	}

	return mysql_use_result(connection);
}

void SQLConnect::updateModelSelected(MYSQL * connection, int modelSelected) {
	//UPDATE `argo`.`argo_app_steps` SET `currentStep`='5' WHERE `id`='1';
	MYSQL_RES *res;	// the results
	MYSQL_ROW row;	// the results row (line by line)

	std::ostringstream buf;
	buf << "UPDATE `argo`.`argo_app_steps` SET `modelSelected`= '" << modelSelected << "'" << " WHERE id = '" << modelSelected << "'";

	string str = buf.str();

	char* s2 = (char *)alloca(str.size() + 1);
	memcpy(s2, str.c_str(), str.size() + 1);
	res = mysql_perform_query(connection, s2);

}

void SQLConnect::updateHasError(MYSQL * connection, int hasError) {
	//UPDATE `argo`.`argo_app_steps` SET `currentStep`='5' WHERE `id`='1';
	MYSQL_RES *res;	// the results
	MYSQL_ROW row;	// the results row (line by line)

	std::ostringstream buf;
	buf << "UPDATE `argo`.`argo_app_steps` SET `hasError`= '" << hasError << "'" << " WHERE id = '" << modelSelected << "'";

	string str = buf.str();

	char* s2 = (char *)alloca(str.size() + 1);
	memcpy(s2, str.c_str(), str.size() + 1);
	res = mysql_perform_query(connection, s2);

}
