#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

int main(int argc, char const *argv[])
{
    doctest::Context context;
    context.applyCommandLine(argc, argv);
    return context.run();
}
