/*
#include <iostream>
#include <cstdlib>
#include <pthread.h>

using namespace std;

#define NUM_THREADS 5

void *PrintHello(void *threadid) {
   long tid;
   tid = (long)threadid;
   cout << "Hello World! Thread ID, " << tid << endl;
   pthread_exit(NULL);
}

int main () {
   pthread_t threads[NUM_THREADS];
   int rc;
   int i;
   
   for( i = 0; i < NUM_THREADS; i++ ) {
      cout << "main() : creating thread, " << i << endl;
      rc = pthread_create(&threads[i], NULL, PrintHello, (void *)i);
      
      if (rc) {
         cout << "Error:unable to create thread," << rc << endl;
         exit(-1);
      }
   }
   pthread_exit(NULL);
}
*/
// thread example
#include <iostream>       // std::cout
#include <thread>         // std::thread 
#include <mutex>          // std::mutex

using namespace std;

std::mutex mtx;           // mutex for critical section

void bar(int x)
{
   mtx.lock();
   cout << "bar " << x <<"\n";
   mtx.unlock();
}

int main() 
{
   //mutex
   //array of threads
   
   int i, numAgt = 30;
   std::thread  agts[numAgt];
   for (i = 0; i < numAgt; i++)
      agts[i] = std::thread(bar,i);

   // synchronize threads:
   for (i = 0; i < numAgt; i++)
      agts[i].join();   // pauses until finishes
   
   std::cout << "Done.\n";

  return 0;
}