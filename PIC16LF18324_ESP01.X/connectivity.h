/* 
 * File:   connectivity.h
 * Author: lesfo
 *
 * Created on December 6, 2024, 8:48 PM
 */

#ifndef CONNECTIVITY_H
#define	CONNECTIVITY_H

#ifdef	__cplusplus
extern "C" {
#endif

// Edit these credentials to make things work.
// It is not secure practice to commit credentials to a repo
// You may wish to:
//    mark this one with .gitignore.
//    git rm --cached <this header file>
//

// NOTE: 4-byte port will ensure no change need be made in GET_PRFX value
//  GET_PRFX includes the actual length of data.

// Some sources say CWJAP_CUR.  Replace values below with local ssid and pw
static unsigned const char CONNECT[] = "AT+CWJAP=\"SSID\",\"PASSWORD\"\r\n";
// Replace 10.0.0.100 with the IP of your server. Replace 8080 with your port
// Be aware of the GET_PRFX string's dependence on length of GET_REQ string.
// The string of X's is a placeholder. You may use whatever web server
// you like but be aware of how that value is replaced.
static unsigned const char CREATE_TCP[] = "AT+CIPSTART=\"TCP\",\"10.0.0.100\",8080\r\n";
static unsigned char GET_REQ[] = "GET /reading/XXXXXXXXXX HTTP/1.1\r\nHost: 10.0.0.100\r\nUser-Agent: PIC16LF18324\r\n\r\n";

#ifdef	__cplusplus
}
#endif

#endif	/* CONNECTIVITY_H */

