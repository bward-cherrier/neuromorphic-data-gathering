import dv_processing as dv

# Trigger data only configuration
config = dv.io.MonoCameraWriter.Config("DVXplorer_sample")
config.addTriggerStream()

# Create the writer instance, it will only have a single trigger output stream
writer = dv.io.MonoCameraWriter("mono_writer_sample.aedat4", config)

# Write 100 triggers
for i in range(100):
    # Generate some monotonically increasing timestamp
    timestamp = i * 1000

    # Single trigger instance, let's say this is some signal from external source
    trigger = dv.Trigger(timestamp, dv.TriggerType.EXTERNAL_GENERATOR_RISING_EDGE)

    # Write the trigger value
    writer.writeTrigger(trigger)
