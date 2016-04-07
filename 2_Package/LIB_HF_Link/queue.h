#ifndef   __queue_H_
#define   __queue_H_

#define QUEUE_SIZE 512

class Queue
{
public:
    Queue();
    unsigned char arr[QUEUE_SIZE];
    int head,tail;
    void Queue_Init(void);
    unsigned char get_data(void);
    void put_data(unsigned char ch);
    unsigned char  empty_check(void);
    unsigned char  full_check(void);

private:
};

#endif

