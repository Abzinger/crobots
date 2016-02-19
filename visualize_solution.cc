// visualize_solution.cc C++11
// Part of the Robot Routing project
// Author: Dirk Oliver Theis

#include "grid.hh"
#include "grid_stat.hh"

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <termios.h>


typedef GridSpace::Read_From_Raw_Data__Grid   Grid;

int main(int argc, const char *argv[]) try
{
    std::cout<<"Robot Router visualize_solution\nPart of the Robot Routing project\n"<<std::endl;

    if (argc!=2) {
        std::cerr<<
            "USAGE: visualize_solution fn\n"
            " where    fn is the name of a file of ``roboute'' type.\n";
        return 1;
    }

    //******************************************************************************************************************************************************
    const char * const filename = argv[1];
    std::cout<<"Opening file ``"<<filename<<"'' for reading..."<<std::flush;
    std::ifstream file {filename};
    file.exceptions(file.exceptions() | std::ios_base::badbit | std::ios_base::failbit);
    std::cout<<"done\nReading grid size from file..."<<std::flush;
    file>>std::ws; // eat whitespace
    while (file.peek()=='#') {
        std::string comment;
        std::getline(file,comment);
        std::cout<<"\n Comment:"<<comment;
        file>>std::ws; // eat whitespace
    }
    short NS,EW;
    file>>NS>>EW;
    std::cout<<"\nDone: dimension are NS="<<NS<<", EW="<<EW<<"\n";
    std::cout<<"Now reading grid data..."<<std::flush;

    std::string tmpstr;

    std::getline(file>>std::ws,tmpstr);
    Grid G {NS,EW,tmpstr};
    std::cout<<"done.\nNow reading t_max..."<<std::flush;
    unsigned t_max;
    file>>t_max;
    std::cout<<"done.\nt_max="<<t_max<<"\n";
    std::cout<<"Now reading the "<<t_max+1<<"grid statuses: "<<std::flush;

    std::vector< GridSpace::Stat_Vector_t > fullsol {t_max+1, G};
    for (unsigned t=0; t<=t_max; ++t) {
        std::getline(file>>std::ws,tmpstr);
        GridSpace::raw_read( &( fullsol[t] ), tmpstr );
        std::cout<<'.'<<std::flush;
    }
    std::cout<<"done. Looks like we're ready to go!"<<std::endl;

    //******************************************************************************************************************************************************

    std::cout<<"Press enter to view solution."<<std::endl;
    std::cin.get();

    struct termios new_kbd_mode;
    struct termios g_old_kbd_mode;

    // put keyboard (stdin) in raw, unbuffered mode
    tcgetattr (0, &g_old_kbd_mode);
    std::memcpy(&new_kbd_mode, &g_old_kbd_mode, sizeof (struct termios));

    new_kbd_mode.c_lflag &= ~(ICANON | ECHO);
    new_kbd_mode.c_cc[VTIME] = 0;
    new_kbd_mode.c_cc[VMIN] = 1;
    tcsetattr (0, TCSANOW, &new_kbd_mode);

    for (unsigned t=0, quit=0; !quit; ) {
        char k = ' ';
        while ( ! (k=='q' || k=='Q') ) {
            std::cout<<"\e[1;1H\e[2J"; // clear screen
            std::cout<<GridSpace::print(fullsol[t]);
            std::cout<<"t="<<t<<"     [n]ext, [p]revious, [q]uit"<<std::endl;

            // wait for (valid) keyboard input
            while (true) {
                k = std::cin.get();
                if ( (k=='n' || k=='N') &&   t < t_max ) {++t; break; }
                if ( (k=='p' || k=='P') &&   t > 0     ) {--t; break; }
                if (  k=='q' || k=='Q'                 )       break;
            }
        } // while not k==q
        std::cout<<"\nREALLY QUIT??? [y]es/*";
        k = std::cin.get();
        if (k=='y' || k=='Y') quit=1;
    } // for  until quit

    // set back into old mode
    tcsetattr (0, TCSANOW, &g_old_kbd_mode);

    //******************************************************************************************************************************************************

    std::cout<<
        "\nRobot Router visualize_solution was brought to you by\n"
        "Algorithms & Theory @ Uni Tartu\n"
        "http://ac.cs.ut.ee\n"
        "Bye-bye."<<std::endl;
} // try/main
catch(const std::exception &stde) {
    std::cout<<std::endl;
    std::cerr<<"The following std::exception was caught by main():\n";
    std::cerr<<stde.what();
    std::cerr<<std::endl;
}
catch(const std::string &stre) {
    std::cout<<std::endl;
    std::cerr<<"The following std::string was caught as an exception by main():\n";
    std::cerr<<stre;
    std::cerr<<std::endl;
}
catch(const char *cstr) {
    std::cout<<std::endl;
    std::cerr<<"The following const-char[] exception was caught as an exception by main():\n";
    std::cerr<<cstr;
    std::cerr<<std::endl;
}
catch(...) {
    std::cout<<std::endl;
    std::cerr<<"Unknown exception caught by main()\n";
}

// main()

// EOF visualize_solution.cc

