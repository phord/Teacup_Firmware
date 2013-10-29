/*
 * data_recorder.h
 */

#ifndef DATA_RECORDER_H_
#define DATA_RECORDER_H_

void recorder_init(const char* filename);
void record_pin(int pin, bool state, uint32_t time);
void add_trace_var(const char* name, int pin);
void record_comment(const char * msg);
void record_comment_stream(char ch);

#endif /* DATA_RECORDER_H_ */
