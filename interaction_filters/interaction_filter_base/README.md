# interaction_filter_base

This package provides a superclass for a generic interaction filter, i.e. admittance filter, direct force filter, etc.

## How to configure

Interaction filters' configuration are retrieved from the node parameters interface.
The initialization process is carried out by the `initialize()` method of the [base class](./include/interaction_filter_base/interaction_filter_base.hpp), which expects the desired filter configuration under the filter name namespace.

Filter parameters can be stored by either declaring them directly as node parameters or by saving them in a `.yaml` file.

The `initialize()` method has to be called with the following parameters:

- `node_params`: the `NodeParametersInterface` instance of the node where the parameters are stored;
- `node_logger`: the `NodeLoggingInterface` instance;
- `filter_name`: a unique name for the filter.

Below a template of the method to call to configure the filter:

```c++
filter->configure(node_params,
                  node_logger,
                  "filter_name",
);
```

## How to use

After initialized, the `update()` method receives a new input, the control period and returns the output command of the filter.

To reset the filter, the `reset()` method can be called. This method is useful when the system state changes significantly.
