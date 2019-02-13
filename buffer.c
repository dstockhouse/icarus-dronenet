/****************************************************************************\
 *
 * File:
 * 	buffer.c
 *
 * Description:
 * 	Buffer struct for storing bytes from an input serial port and handlers
 * 	for managing the buf
 *
 * Author:
 * 	David Stockhouse
 *
 * Revision 0.1
 * 	Last edited 2/13/2019
 *
\***************************************************************************/

#include "buffer.h"


/**** Function BufferAdd ****
 *
 * Adds a value to a BYTE_BUFFER instance
 *
 * Arguments: 
 * 	buf      - Pointer to BYTE_BUFFER instance to empty
 * 	data     - Single byte to add to array
 *
 * Return value:
 * 	Returns number of elements added (1 or 0)
 *      If buf is NULL returns -1
 */
int BufferAdd(BYTE_BUFFER *buf, unsigned char data) {

	// Exit if buffer pointer invalid
	if(buf == NULL) {
		return -1;
	}

	// Ensure buffer is not full
	if(buf->length < BYTE_BUFFER_LEN - 1) {

		// Increase length and put element at end of buffer
		buf->length += 1;
		buf->buffer[buf->length] = data;

		// Added one element
		return 1;

	} // else

	// Didn't add anything
	return 0;

} // BufferAdd(BYTE_BUFFER *, unsigned char)


/**** Function BufferAddArray ****
 *
 * Adds a supplied array of values to a BYTE_BUFFER instance
 *
 * Arguments: 
 * 	buf      - Pointer to BYTE_BUFFER instance to empty
 * 	data     - Array containing data to add to array
 * 	numToAdd - Number of elements to remove from buffer
 *
 * Return value:
 * 	On success returns number of elements successfully added
 *      If buf is NULL returns -1
 */
int BufferAddArray(BYTE_BUFFER *buf, unsigned char *data, int numToAdd) {

	int i, numAdded = 0;

	if(buf == NULL) {
		return -1;
	}

	// Add as many elements as will fit
	for(i = 0; i < numToAdd; i++) {

		// Use above function to add one element and increment if successful
		if(BufferAdd(buf, data[i]) > 0) {
			numAdded++;
		}
	}

	// Return number successfully added
	return numAdded;

} // BufferAddArray(BYTE_BUFFER *, unsigned char *, int)


/**** Function BufferRemove ****
 *
 * Removes a number of values from the start of a BYTE_BUFFER instance
 *
 * Arguments: 
 * 	buf         - Pointer to BYTE_BUFFER instance to empty
 * 	numToRemove - Number of elements to remove from buffer
 *
 * Return value:
 * 	On success returns new buffer length
 *      If buf is NULL returns -1
 */
int BufferRemove(BYTE_BUFFER *buf, int numToRemove) {

	int i;

	if(buf == NULL) {
		return -1;
	}

	// Always remove from index 0 and shift elements backward
	for(i = 0; i + numToRemove < buf->length; i++) {
		buf->buffer[i] = buf->buffer[i + numToRemove];
	}

	// Update new length with the number of elements removed (how far the loop made it)
	buf->length = i;

	// Return new length of buffer
	return buf->length;

} // BufferRemove(BYTE_BUFFER *, int)


/**** Function BufferEmpty ****
 *
 * Empties a BYTE_BUFFER instance
 *
 * Arguments: 
 * 	buf - Pointer to BYTE_BUFFER instance to empty
 *
 * Return value:
 * 	On success returns 0
 *      If buf is NULL returns -1
 */
int BufferEmpty(BYTE_BUFFER *buf) {

	// Exit if buffer pointer invalid
	if(buf == NULL) {
		// Invalid buffer is treated as full
		return -1;
	}

	// Leave data in buffer but consider data to be invalid
	buf->length = 0;

	// Return on success
	return 0;

} // BufferEmpty(BYTE_BUFFER *)

