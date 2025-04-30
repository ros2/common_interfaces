# Result code and message for service calls.
# Note that additional results for specific services can defined within them using values above 100.

uint8 RESULT_FEATURE_UNSUPPORTED   = 0    # Feature is not supported by the simulator, check GetSimulatorFeatures.
                                          # While feature support can sometimes be deduced from presence of corresponding
                                          # service / action interface, in other cases it is about supporting certain
                                          # call parameters, formats and options within interface call.
uint8 RESULT_OK                    = 1
uint8 RESULT_NOT_FOUND             = 2    # No match for input (such as when entity name does not exist).
uint8 RESULT_INCORRECT_STATE       = 3    # Simulator is in an incorrect state for this interface call, e.g. a service
                                          # requires paused state but the simulator is not paused.
uint8 RESULT_OPERATION_FAILED      = 4    # Request could not be completed successfully even though feature is supported
                                          # and the input is correct; check error_message for details.
                                          # Implementation rule: check extended codes for called service
                                          #  (e.g. SpawnEntity) to provide a return code which is more specific.

uint8 result                              # Result to be checked on return from service interface call
string error_message                      # Additional error description when useful.
