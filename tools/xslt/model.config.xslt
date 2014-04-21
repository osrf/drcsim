<xsl:stylesheet version="1.0"
 xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
 <xsl:output omit-xml-declaration="yes" indent="yes"/>
 <xsl:param name="model_name"/>
 <xsl:param name="model_sdf"/>

 <xsl:template match="model/name">
   <name><xsl:value-of select="$model_name"/></name>
 </xsl:template>
 <xsl:template match="model/description">
   <description><xsl:value-of select="$model_name"/></description>
 </xsl:template>
 <xsl:template match="model/sdf">
   <sdf><xsl:value-of select="$model_sdf"/></sdf>
 </xsl:template>

 <xsl:template match="node()|@*">
  <xsl:copy>
   <xsl:apply-templates select="node()|@*"/>
  </xsl:copy>
 </xsl:template>

</xsl:stylesheet>
