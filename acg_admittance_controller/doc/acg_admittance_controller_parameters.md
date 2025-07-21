# Acg Admittance Controller Parameters

Default Config
```yaml
acg_admittance_controller:
  ros__parameters:
    force_limitation_enabled: false
    logging_enabled: false
    reference_scaling_gain: ''

```

## force_limitation_enabled

Specifies whether the force limitation is enabled or not.

* Type: `bool`
* Default Value: false

## reference_scaling_gain

Specifies the scaling gain for the position reference.

* Type: `double`

*Constraints:*
 - parameter must be within bounds 0.0

*Additional Constraints:*



## logging_enabled

Specifies whether logging is enabled or not.

* Type: `bool`
* Default Value: false

