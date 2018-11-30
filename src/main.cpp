#include <iostream>
#include <aris.h>
#include<string>


int main()
{
    std::cout << "start testing IO board" << std::endl;

    aris::control::EthercatMaster mst;

    mst.scan();

    std::cout<<"slave num:"<<mst.slavePool().size()<<std::endl;

    mst.setControlStrategy([&]()
    {
        static int count{ 0 };
        static std::uint8_t value{ 0x01 };

        if (++count % 1000 == 0)
        {
            value = value << 1;
            if(value == 0) value = 0x01;
            mst.mout() << "count:" << std::dec << count << std::endl;
            mst.ecSlavePool().at(1).writePdo(0x7001,0x02,&value,8);

            //value = ~value;
        }
    });

    mst.start();

    std::this_thread::sleep_for(std::chrono::seconds(100));

    mst.stop();

	return 0;
}
