# Nested Functions in C and C++

## GNU C Specific
Nested function definition
```
void func1() {
    void func2() {
        // ...
    }
}
```
is not valid in ANSI C. It is [a language extension supported by GCC](https://gcc.gnu.org/onlinedocs/gcc/Nested-Functions.html). You will find this feature available to you in STM32CubeIDE since GCC is used. Avoid overfiting yourself to any compiler specific language features while learning C.

## Limited Scope of The Inner Function
The function `func2` in the example will not be available for external or internal linkage beyond the scope of `func1`.

For example, if you define a STM32 HAL callback function `HAL_TIM_PeriodElapsedCallback` within `main`, the linker will not be able to see it and replace the original weak definition. Thus your ISR cannot work as intended.

## Similar Feature in C++
C++ does not support nested functions directly either. But lambdas and static functions in a nested class/struct definition can achieve the same effect.

## Reading
- [Stack overflow: Nested function in C](https://stackoverflow.com/questions/2608158/nested-function-in-c)
- [Stack overflow: Can we have functions inside functions in C++?](https://stackoverflow.com/questions/4324763/can-we-have-functions-inside-functions-in-c)
- [Stack overflow: Passing capturing lambda as function pointer](https://stackoverflow.com/questions/28746744/passing-capturing-lambda-as-function-pointer)
