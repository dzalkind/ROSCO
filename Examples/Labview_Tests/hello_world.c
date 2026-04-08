#include <stdio.h>
#include <string.h>

// Export macro — required on Windows
#ifdef _WIN32
  #include <direct.h>
  #define EXPORT __declspec(dllexport)
  #define GETCWD _getcwd
#else
  #include <unistd.h>
  #define EXPORT
  #define GETCWD getcwd
#endif

extern void hello_fortran(char* input_string);

EXPORT void hello_world(char* output, int max_len, const char* logName) {
  char cwd[FILENAME_MAX];  
  GETCWD(cwd, sizeof(cwd));

  const char* message = cwd; // Message to log, which is the current working directory
  FILE* log_file;
  
  log_file = fopen(logName, "a");
  fprintf(log_file, "%s\n", message);
  fclose(log_file);

  char input_string[100] = "hello_c";

  // Delete any Fortran scratch files left from a previous run.
  remove("fort.10");

  hello_fortran(input_string);

  strncpy(output, input_string, max_len - 1); // Copy the message to the output buffer, to check cwd in LabVIEW
  output[max_len - 1] = '\0';

  log_file = fopen(logName, "a");
  fprintf(log_file, "Fortran test string: %s\n", input_string);
  fclose(log_file);


}