<xsl:stylesheet version="1.0"
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
  <xsl:output omit-xml-declaration="yes" indent="yes"/>
  <xsl:param name="old_model_name"/>
  <xsl:param name="new_model_name"/>

  <!-- String replace template copied from
       http://stackoverflow.com/questions/7520762/xslt-1-0-string-replace-function -->
  <xsl:template name="replace-string">
    <xsl:param name="text"/>
    <xsl:param name="replace"/>
    <xsl:param name="with"/>
    <xsl:choose>
      <xsl:when test="contains($text,$replace)">
        <xsl:value-of select="substring-before($text,$replace)"/>
        <xsl:value-of select="$with"/>
        <xsl:call-template name="replace-string">
          <xsl:with-param name="text"
select="substring-after($text,$replace)"/>
          <xsl:with-param name="replace" select="$replace"/>
          <xsl:with-param name="with" select="$with"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:value-of select="$text"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- Identity copy -->
  <xsl:template match="node()|@*">
    <xsl:copy>
      <xsl:apply-templates select="node()|@*"/>
    </xsl:copy>
  </xsl:template>

  <!-- Remove plugin tags -->
  <xsl:template match="plugin"/>

  <!-- Modify the filename attribute of mesh tags -->
  <xsl:template match="robot/link/*/geometry/mesh">
    <xsl:copy>
      <xsl:copy-of select="@*"/>
      <xsl:attribute name="filename">
        <xsl:call-template name="replace-string">
          <xsl:with-param name="text">
            <xsl:value-of select="@filename"/>
          </xsl:with-param>
          <xsl:with-param name="replace" select="$old_model_name"/>
          <xsl:with-param name="with" select="$new_model_name"/>
        </xsl:call-template>
      </xsl:attribute>
    </xsl:copy>
  </xsl:template>

</xsl:stylesheet>
