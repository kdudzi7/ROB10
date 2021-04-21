#include <chrono>
#include <thread>
#include <iostream>
using namespace std;
using namespace std::literals;
using clock_type = std::chrono::high_resolution_clock;

void doSomething() 
{
	cout << "\a";
    cout << "Beep" << endl;
   
}



void timeQuick()
{
	auto when_started = clock_type::now(); 
    auto target_time = when_started + 100ms;
    for (int i = 0; i < 10; i++) {
        doSomething();
        std::this_thread::sleep_until(target_time);
        target_time += 100ms;
    }
    auto now = clock_type::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - when_started).count() << "ms\n";

}

void timeMedium()
{
	auto when_started = clock_type::now(); 
    auto target_time = when_started + 1000ms;
    for (int i = 0; i < 10; i++) {
        doSomething();
        std::this_thread::sleep_until(target_time);
        target_time += 1000ms;
    }
    auto now = clock_type::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - when_started).count() << "ms\n";

}

void timeSlow()
{
	auto when_started = clock_type::now(); 
    auto target_time = when_started + 10000ms;
    for (int i = 0; i < 10; i++) {
        doSomething();
        std::this_thread::sleep_until(target_time);
        target_time += 10000ms;
    }
    auto now = clock_type::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(now - when_started).count() << "ms\n";

}
int main()
{
	int whichTimer = 3;
	if(whichTimer == 1)
	{
		timeSlow();
	}
	if(whichTimer == 2)
	{
		timeMedium();
	}
	if(whichTimer == 3)
	{
		timeQuick();
	}
}