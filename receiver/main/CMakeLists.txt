set(EXTRA_COMPONENT_DIRS ../components/lora)
idf_component_register(SRCS "test_fous.c" "./Message/impl/Message.c"
                    INCLUDE_DIRS "." "./Message")
