SECTION_PROLOGUE(.got,,)
{
    *(.got.plt)
    *(.got)
} GROUP_LINK_IN(RAMABLE_REGION)