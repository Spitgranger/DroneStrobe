set(EXTRA_COMPONENT_DIRS ../components/lora)
idf_component_register(SRCS "receiver.c" "./Message/impl/Message.c"
                    INCLUDE_DIRS "." "./Message")
