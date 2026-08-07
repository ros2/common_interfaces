# Result code and message for service calls.
# Note that additional codes for specific services can defined within the service definition.
# For example, you could define a service CalculateSquareRoot.srv as
# float64 value
# ---
# int32 CODE_NEGATIVE_VALUE = -110
# std_msgs/Result result
# float64 root
#
# If a negative value is provided, the server could return the generic CODE_INVALID_ARGUMENT
# as result.code, or it could provide more context with CODE_NEGATIVE_VALUE.
#
# As a general guideline,
# Positive values are successes
# Negative values are failures
#       Values between -1 and -100 are for some generic high-level failures
#       Values between -101 and -1000 are for client (caller) errors (a la HTTP 4XX errors)
#       Values between -1001 and -2000 are for server errors (a la HTTP 5XX errors)

int32 CODE_UNKNOWN                   =   0
int32 CODE_OK                        =    1

int32 CODE_GENERAL_FAILURE           =  -1

# Client Errors
int32 CODE_CANCELLED                 =  -101
int32 CODE_INVALID_ARGUMENT          =  -102
int32 CODE_NOT_FOUND                 =  -103
int32 CODE_ALREADY_EXISTS            =  -104
int32 CODE_PERMISSION_DENIED         =  -105
int32 CODE_FAILED_PRECONDITION       =  -106
int32 CODE_OUT_OF_RANGE              =  -107

# Server Errors
int32 CODE_INTERNAL                  = -1000
int32 CODE_UNIMPLEMENTED             = -1001
int32 CODE_UNAVAILABLE               = -1002
int32 CODE_DEADLINE_EXCEEDED         = -1003

int32 code                                # Code number to be checked on return from service interface call
string error_message                      # Additional error description when useful.
